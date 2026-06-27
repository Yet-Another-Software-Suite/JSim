package jsim.forces;

import jsim.dynamics.RigidBody;

/**
 * A source of continuous force or torque applied to one or more bodies each tick.
 *
 * <p>Implementations call {@link RigidBody#applyForce} and/or {@link RigidBody#applyTorque}
 * directly. The world invokes {@link #apply} on every registered generator after clearing
 * accumulators and before running broadphase.
 */
@FunctionalInterface
public interface ForceGenerator {
    /**
     * Apply forces/torques to whichever bodies this generator influences.
     *
     * @param dt the fixed simulation timestep in seconds
     */
    void apply(double dt);
}
