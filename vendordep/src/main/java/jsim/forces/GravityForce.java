package jsim.forces;

import java.util.List;
import jsim.core.SimConstants;
import jsim.dynamics.RigidBody;
import jsim.dynamics.RigidBodyFlags;

/**
 * Applies gravitational acceleration to all dynamic, non-static bodies.
 *
 * <p>Gravity is uniform (not per-body — attach a custom {@link ForceGenerator} for buoyancy
 * or other per-body gravity modifications).
 */
public final class GravityForce implements ForceGenerator {
    private final List<RigidBody> bodies;
    private final double gx, gy, gz;

    /**
     * Default gravity pointing in −Z (WPILib field convention).
     *
     * @param bodies all bodies to apply gravity to
     */
    public GravityForce(List<RigidBody> bodies) {
        this(bodies, SimConstants.GRAVITY_X, SimConstants.GRAVITY_Y, SimConstants.GRAVITY_Z);
    }

    /**
     * Gravity with a custom acceleration vector.
     *
     * @param bodies all bodies to apply gravity to
     * @param gx     gravitational acceleration X component (m/s²)
     * @param gy     gravitational acceleration Y component (m/s²)
     * @param gz     gravitational acceleration Z component (m/s²)
     */
    public GravityForce(List<RigidBody> bodies, double gx, double gy, double gz) {
        this.bodies = bodies;
        this.gx = gx; this.gy = gy; this.gz = gz;
    }

    @Override
    public void apply(double dt) {
        for (RigidBody body : bodies) {
            if (body.isStatic()) continue;
            if (RigidBodyFlags.isSet(body.flags, RigidBodyFlags.NO_GRAVITY)) continue;
            if (body.invMass <= 0) continue;
            double mass = 1.0 / body.invMass;
            body.applyForce(gx * mass, gy * mass, gz * mass);
        }
    }
}
