package frc.jsim.forces;

import frc.jsim.dynamics.RigidBody;

/**
 * Force generator that represents a motor, pneumatic, or other actuator.
 *
 * <p>Robot code sets the desired force/torque via {@link #setForce} and {@link #setTorque}
 * each control loop; the simulation picks up the latest values on the next physics tick.
 * All values are in world frame unless the body does not rotate (e.g. using FIXED_ROTATION).
 *
 * <p>To model a drivetrain motor, create one ActuatorForce per wheel and set the wheel's
 * traction force each tick based on motor output voltage and the drive model.
 */
public final class ActuatorForce implements ForceGenerator {
    private final RigidBody body;

    private double fx, fy, fz;
    private double tx, ty, tz;

    /** Optional application point in world frame. If NaN, force is applied at CoM. */
    private double appX = Double.NaN, appY, appZ;

    public ActuatorForce(RigidBody body) {
        this.body = body;
    }

    /** Set the world-frame force to apply at the body's centre of mass (N). */
    public void setForce(double fx, double fy, double fz) {
        this.fx = fx; this.fy = fy; this.fz = fz;
        this.appX = Double.NaN;
    }

    /** Set the world-frame torque to apply directly (N·m). */
    public void setTorque(double tx, double ty, double tz) {
        this.tx = tx; this.ty = ty; this.tz = tz;
    }

    /** Set the force and its world-space application point (generates torque automatically). */
    public void setForceAtPoint(double fx, double fy, double fz,
                                 double px, double py, double pz) {
        this.fx = fx; this.fy = fy; this.fz = fz;
        this.appX = px; this.appY = py; this.appZ = pz;
    }

    /** Clear all actuator forces and torques (e.g. on disable). */
    public void zero() {
        fx = fy = fz = tx = ty = tz = 0;
        appX = Double.NaN;
    }

    @Override
    public void apply(double dt) {
        if (body.isStatic()) return;
        if (Double.isNaN(appX)) {
            body.applyForce(fx, fy, fz);
        } else {
            body.applyForceAtPoint(fx, fy, fz, appX, appY, appZ);
        }
        body.applyTorque(tx, ty, tz);
    }
}
