package frc.jsim.api;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import frc.jsim.core.PhysicsWorld;
import frc.jsim.core.SimConstants;
import frc.jsim.forces.DragForce;
import frc.jsim.forces.ForceGenerator;
import frc.jsim.forces.MagnusForce;

/**
 * Top-level public API for the JSim physics simulation.
 *
 * <p>Create a world, add bodies, and call {@link #step()} each robot loop iteration:
 * <pre>{@code
 * SimWorld world = new SimWorld();
 *
 * // Static floor
 * world.addBody(new SimBodyBuilder("Floor")
 *     .planeCollider(0, 0, 1, 0)
 *     .material(Material.CARPET));
 *
 * // Game piece
 * SimBody ball = world.addBody(new SimBodyBuilder("Ball")
 *     .position(1, 2, 0.1)
 *     .mass(0.24)
 *     .sphereCollider(0.085)
 *     .material(Material.RUBBER));
 *
 * // In your simulationPeriodic():
 * world.step();
 * Pose3d pose = ball.getPose();
 * }</pre>
 */
public final class SimWorld {
    private final PhysicsWorld physics;
    private final double dt;

    private final List<SimBody> simBodies = new ArrayList<>();

    /** Creates a world with the default timestep and solver iteration count. */
    public SimWorld() {
        this(SimConstants.DEFAULT_DT, SimConstants.DEFAULT_SOLVER_ITERATIONS);
    }

    /**
     * Creates a world with a custom fixed timestep and the default solver iteration count.
     *
     * @param fixedTimestepSeconds the physics timestep in seconds
     */
    public SimWorld(double fixedTimestepSeconds) {
        this(fixedTimestepSeconds, SimConstants.DEFAULT_SOLVER_ITERATIONS);
    }

    /**
     * Creates a world with a custom fixed timestep and solver iteration count.
     *
     * @param fixedTimestepSeconds the physics timestep in seconds
     * @param solverIterations number of constraint-solver iterations per step
     */
    public SimWorld(double fixedTimestepSeconds, int solverIterations) {
        this.dt = fixedTimestepSeconds;
        this.physics = new PhysicsWorld(solverIterations);
    }

    // Body management

    /**
     * Build a body from the supplied builder, register it in the world, and
     * return the high-level {@link SimBody} handle.
     *
     * @param builder configured body builder
     * @return the newly created {@link SimBody}
     */
    public SimBody addBody(SimBodyBuilder builder) {
        SimBody sb = builder.build();
        physics.addBody(sb.body);
        physics.addForceGenerator(sb.actuator);
        simBodies.add(sb);
        return sb;
    }

    /**
     * Remove a previously added body from the simulation.
     *
     * @param sb the body to remove
     */
    public void removeBody(SimBody sb) {
        physics.removeBody(sb.body);
        physics.removeForceGenerator(sb.actuator);
        simBodies.remove(sb);
    }

    /**
     * Read-only list of all registered bodies in insertion order.
     *
     * @return unmodifiable list of all bodies
     */
    public List<SimBody> getBodies() {
        return Collections.unmodifiableList(simBodies);
    }

    // Force generators

    /**
     * Register a custom force generator (drag, magnus, spring, custom motor model, etc.).
     * The generator is called every tick after built-in gravity.
     *
     * @param fg the force generator to register
     */
    public void addForceGenerator(ForceGenerator fg) {
          physics.addForceGenerator(fg);
    }

    /**
     * Unregister a previously added force generator.
     *
     * @param fg the force generator to remove
     */
    public void removeForceGenerator(ForceGenerator fg) {
        physics.removeForceGenerator(fg);
    }

    /**
     * Create and register a {@link DragForce} for the given body.
     *
     * <p>Example usage:
     * <pre>{@code
     * DragForce drag = world.addDragForce(ball, 0.1, 0.05);
     * // later, to remove:
     * world.removeForceGenerator(drag);
     * }</pre>
     *
     * @param body            the body to apply drag to
     * @param linearDamping   linear drag coefficient (N·s/m)
     * @param angularDamping  rotational drag coefficient (N·m·s/rad)
     * @return the registered {@link DragForce}
     */
    public DragForce addDragForce(SimBody body, double linearDamping, double angularDamping) {
        DragForce df = new DragForce(body.body, linearDamping, angularDamping);
        physics.addForceGenerator(df);
        return df;
    }

    /**
     * Create and register a {@link MagnusForce} for the given spinning body.
     *
     * <p>The Magnus effect produces a lift force perpendicular to both the spin axis and
     * the velocity vector, causing curved flight for spinning game pieces.
     *
     * @param body              the spinning body to apply Magnus lift to
     * @param magnusCoefficient Magnus lift coefficient k (kg); typically 0.1–2.0 for FRC game pieces
     * @return the registered {@link MagnusForce}
     */
    public MagnusForce addMagnusForce(SimBody body, double magnusCoefficient) {
        MagnusForce mf = new MagnusForce(body.body, magnusCoefficient);
        physics.addForceGenerator(mf);
        return mf;
    }

    // Simulation control
    /**
     * Advance the simulation by the fixed timestep configured at construction time.
     * Call once per robot loop iteration from {@code simulationPeriodic()}.
     */
    public void step() {
        physics.step(dt);
    }
    /**
     * Advance by an explicit timestep (useful for tests or variable-rate callers).
     * Determinism is only guaranteed when {@code dt} is the same every call.
     *
     * @param dtSeconds timestep in seconds
     */
    public void step(double dtSeconds) {
        physics.step(dtSeconds);
    }
    /**
     * The fixed timestep this world was configured with (seconds).
     *
     * @return timestep in seconds
     */
    public double getTimestep() { return dt; }

    // Convenience finders

    /**
     * Find the first body with the given name, or {@code null} if none.
     *
     * @param name the body name to search for
     * @return the matching body, or {@code null}
     */
    public SimBody findBody(String name) {
        for (SimBody sb : simBodies) {
            if (sb.getName().equals(name)) return sb;
        }
        return null;
    }
    /**
     * Find the robot body registered with the given alliance-station identity,
     * or {@code null} if no body was added with that {@link RobotId}.
     *
     * @param id the alliance-station identity to look up
     * @return the matching robot body, or {@code null}
     */
    public SimBody findRobot(RobotId id) {
        for (SimBody sb : simBodies) {
            if (id.equals(sb.getRobotId())) return sb;
        }
        return null;
    }
}
