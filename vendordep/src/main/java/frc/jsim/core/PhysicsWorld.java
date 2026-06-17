package frc.jsim.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import frc.jsim.collision.BroadPhase;
import frc.jsim.collision.CollisionPair;
import frc.jsim.collision.ContactPoint;
import frc.jsim.collision.NarrowPhase;
import frc.jsim.dynamics.RigidBody;
import frc.jsim.dynamics.RigidBodyFlags;
import frc.jsim.forces.ForceGenerator;
import frc.jsim.forces.GravityForce;
import frc.jsim.solver.ImpulseSolver;

/**
 * The central simulation world.
 *
 * <p>Advances through a strict deterministic pipeline on each call to {@link #step}:
 * <ol>
 *   <li>Clear force/torque accumulators on all bodies.
 *   <li>Apply all registered force generators (gravity, drag, magnus, actuators).
 *   <li>Broadphase — AABB overlap test, produces candidate pairs.
 *   <li>Narrowphase — geometry-accurate contact generation.
 *   <li>Solve — iterative impulse projection (normal + friction).
 *   <li>Integrate — semi-implicit Euler: v += a·dt, x += v·dt; quaternion update.
 *   <li>Positional correction — Baumgarte (baked into solver bias).
 *   <li>Update derived state — AABB refresh, quaternion normalisation, inertia refresh.
 * </ol>
 *
 * <p>The API layer ({@link frc.jsim.api.SimWorld}) owns this object and delegates to it.
 * External code should not reference this class directly.
 */
public final class PhysicsWorld {
    private final List<RigidBody> bodies = new ArrayList<>();
    private final List<ForceGenerator> forceGenerators = new ArrayList<>();

    private final List<CollisionPair> pairBuffer  = new ArrayList<>();
    private final List<ContactPoint>  contacts     = new ArrayList<>();

    private final ImpulseSolver solver;
    private final GravityForce gravityForce;

    private int nextId = 0;

    /** Creates a world with default solver iteration count. */
    public PhysicsWorld() {
        this(SimConstants.DEFAULT_SOLVER_ITERATIONS);
    }

    /**
     * Creates a world with the given solver iteration count.
     *
     * @param solverIterations number of impulse-solver iterations per tick
     */
    public PhysicsWorld(int solverIterations) {
        this.solver       = new ImpulseSolver(solverIterations);
        this.gravityForce = new GravityForce(bodies);
        forceGenerators.add(gravityForce);
    }

    // Registration

    /**
     * Add a body to the world and assign it a unique ID.
     *
     * @param body the body to register
     */
    public void addBody(RigidBody body) {
        body.id = nextId++;
        bodies.add(body);
        updateDerivedState(body);
    }

    /**
     * Remove a body from the world.
     *
     * @param body the body to remove
     */
    public void removeBody(RigidBody body) {
        bodies.remove(body);
    }

    /**
     * Register an additional force generator (e.g. actuator, drag, magnus).
     *
     * @param fg the force generator to add
     */
    public void addForceGenerator(ForceGenerator fg) {
        forceGenerators.add(fg);
    }

    public void removeForceGenerator(ForceGenerator fg) {
        forceGenerators.remove(fg);
    }

    /** Read-only view of all bodies for sensors/tracking. */
    public List<RigidBody> getBodies() {
        return Collections.unmodifiableList(bodies);
    }

    // Main tick pipeline

    /**
     * Advance the simulation by exactly {@code dt} seconds.
     * Always use a fixed timestep for determinism.
     */
    public void step(double dt) {
        // 1. Clear accumulators
        for (RigidBody b : bodies) b.clearAccumulators();

        // 2. Apply forces
        for (ForceGenerator fg : forceGenerators) fg.apply(dt);

        // 3. Broadphase
        BroadPhase.findPairs(bodies, pairBuffer);

        // 4. Narrowphase
        NarrowPhase.generateContacts(pairBuffer, contacts);

        // 5. Solve constraints (includes positional correction via Baumgarte bias)
        solver.solve(contacts, dt);

        // 6. Integrate (semi-implicit Euler)
        for (RigidBody b : bodies) integrate(b, dt);

        // 7+8. Update derived state (AABB, world inertia, quaternion normalisation)
        for (RigidBody b : bodies) updateDerivedState(b);
    }

    // Integration — semi-implicit Euler

    private static void integrate(RigidBody b, double dt) {
        if (b.isStatic()) return;

        // Linear: a = F/m, then v += a*dt, pos += v*dt (semi-implicit: uses updated v)
        double ax = b.forceX * b.invMass;
        double ay = b.forceY * b.invMass;
        double az = b.forceZ * b.invMass;
        b.velX += ax * dt;
        b.velY += ay * dt;
        b.velZ += az * dt;
        b.posX += b.velX * dt;
        b.posY += b.velY * dt;
        b.posZ += b.velZ * dt;

        if (RigidBodyFlags.isSet(b.flags, RigidBodyFlags.FIXED_ROTATION)) return;

        // Angular: α = I⁻¹ · τ, then ω += α*dt
        double[] alpha = Mat3.mulVec(b.invIWorld, new double[]{b.torqueX, b.torqueY, b.torqueZ});
        b.omX += alpha[0] * dt;
        b.omY += alpha[1] * dt;
        b.omZ += alpha[2] * dt;

        // Quaternion integration: dq/dt = 0.5 * [0, ω] * q
        // Using the update: q += 0.5 * dt * omega_pure_quat * q
        double half = 0.5 * dt;
        // omega_pure_quat = (0, omX, omY, omZ)
        // (0,ω) * q = (−ω·q_xyz, ω×q_xyz + 0·q_xyz + ω*q_w)
        // Expanding full quaternion product (0,ω) * (w,x,y,z):
        //   w' = −ωx·x − ωy·y − ωz·z
        //   x' =  ωx·w + ωy·z − ωz·y
        //   y' = −ωx·z + ωy·w + ωz·x
        //   z' =  ωx·y − ωy·x + ωz·w
        double dw = -b.omX * b.qX - b.omY * b.qY - b.omZ * b.qZ;
        double dx = b.omX * b.qW + b.omY * b.qZ - b.omZ * b.qY;
        double dy = -b.omX * b.qZ + b.omY * b.qW + b.omZ * b.qX;
        double dz = b.omX * b.qY - b.omY * b.qX + b.omZ * b.qW;

        b.qW += half * dw;
        b.qX += half * dx;
        b.qY += half * dy;
        b.qZ += half * dz;
        // Normalise (prevents quaternion drift)
        b.normalizeQuaternion();
    }

    // Derived state update

    private static void updateDerivedState(RigidBody b) {
        // Refresh world-frame inverse inertia from updated orientation
        b.refreshWorldInertia();
        // Refresh AABB for next broadphase
        if (b.collider != null) {
            b.collider.computeAABB(b.posX, b.posY, b.posZ, b.qW, b.qX, b.qY, b.qZ, b.aabb);
        }
    }
}
