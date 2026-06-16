package frc.jsim.api;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import frc.jsim.core.PhysicsWorld;
import frc.jsim.core.SimConstants;
import frc.jsim.forces.ForceGenerator;

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

    public SimWorld() {
        this(SimConstants.DEFAULT_DT, SimConstants.DEFAULT_SOLVER_ITERATIONS);
    }

    public SimWorld(double fixedTimestepSeconds) {
        this(fixedTimestepSeconds, SimConstants.DEFAULT_SOLVER_ITERATIONS);
    }

    public SimWorld(double fixedTimestepSeconds, int solverIterations) {
        this.dt = fixedTimestepSeconds;
        this.physics = new PhysicsWorld(solverIterations);
    }

    // ------------------------------------------------------------------
    // Body management
    // ------------------------------------------------------------------

    /**
     * Build a body from the supplied builder, register it in the world, and
     * return the high-level {@link SimBody} handle.
     */
    public SimBody addBody(SimBodyBuilder builder) {
        SimBody sb = builder.build();
        physics.addBody(sb.body);
        physics.addForceGenerator(sb.actuator);
        simBodies.add(sb);
        return sb;
    }

    /** Remove a previously added body from the simulation. */
    public void removeBody(SimBody sb) {
        physics.removeBody(sb.body);
        physics.removeForceGenerator(sb.actuator);
        simBodies.remove(sb);
    }

    /** Read-only list of all registered bodies in insertion order. */
    public List<SimBody> getBodies() {
        return Collections.unmodifiableList(simBodies);
    }

    // ------------------------------------------------------------------
    // Force generators
    // ------------------------------------------------------------------

    /**
     * Register a custom force generator (drag, magnus, spring, custom motor model, etc.).
     * The generator is called every tick after built-in gravity.
     */
    public void addForceGenerator(ForceGenerator fg) {
        physics.addForceGenerator(fg);
    }

    public void removeForceGenerator(ForceGenerator fg) {
        physics.removeForceGenerator(fg);
    }

    // ------------------------------------------------------------------
    // Simulation control
    // ------------------------------------------------------------------

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
     */
    public void step(double dtSeconds) {
        physics.step(dtSeconds);
    }

    /** The fixed timestep this world was configured with (seconds). */
    public double getTimestep() { return dt; }

    // ------------------------------------------------------------------
    // Convenience finders
    // ------------------------------------------------------------------

    /** Find the first body with the given name, or {@code null} if none. */
    public SimBody findBody(String name) {
        for (SimBody sb : simBodies) {
            if (sb.getName().equals(name)) return sb;
        }
        return null;
    }
}
