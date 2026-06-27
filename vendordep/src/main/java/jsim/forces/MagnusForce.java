package jsim.forces;

import jsim.core.Vec3;
import jsim.dynamics.RigidBody;

/**
 * Magnus force for spinning projectiles: F_magnus = k · (ω × v).
 *
 * <p>The cross product of angular velocity with linear velocity produces a lift force
 * perpendicular to the spin axis, causing curved flight. The Magnus coefficient {@code k}
 * (kg/rad·m) is empirically determined per object (typically 0.1–2.0 for FRC game pieces).
 */
public final class MagnusForce implements ForceGenerator {
    private final RigidBody body;
    /** Magnus lift coefficient (kg). */
    private final double k;

    /**
     * Create a Magnus force generator for the given body.
     *
     * @param body              the spinning body to apply Magnus lift to
     * @param magnusCoefficient Magnus lift coefficient k (kg)
     */
    public MagnusForce(RigidBody body, double magnusCoefficient) {
        this.body = body;
        this.k = magnusCoefficient;
    }

    @Override
    public void apply(double dt) {
        if (body.isStatic()) return;
        // F = k * (ω × v)
        double[] omega = Vec3.of(body.omX, body.omY, body.omZ);
        double[] vel   = Vec3.of(body.velX, body.velY, body.velZ);
        double[] f     = Vec3.scale(Vec3.cross(omega, vel), k);
        body.applyForce(f[0], f[1], f[2]);
    }
}
