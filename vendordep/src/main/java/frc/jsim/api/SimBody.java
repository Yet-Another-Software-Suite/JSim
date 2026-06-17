package frc.jsim.api;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.jsim.collision.ColliderShape;
import frc.jsim.dynamics.RigidBody;
import frc.jsim.forces.ActuatorForce;
import frc.jsim.material.Material;

/**
 * High-level handle to a simulated rigid body.
 *
 * <p>All reads and writes use WPILib geometry types. Physics internals are kept
 * inside {@link RigidBody} and are not directly exposed.
 *
 * <p>Obtain instances from {@link SimWorld#addBody(SimBodyBuilder)}.
 */
public final class SimBody {
    final RigidBody body;
    final ActuatorForce actuator;
    private final RobotId robotId;

    SimBody(RigidBody body, ActuatorForce actuator, RobotId robotId) {
        this.body = body;
        this.actuator = actuator;
        this.robotId = robotId;
    }

    // Identity

    public int getId() { return body.id; }
    public String getName() { return body.name; }

    /**
     * The FRC alliance-station identity of this body, or {@code null} if it
     * was not registered with {@link SimBodyBuilder#robotId(RobotId)}.
     */
    public RobotId getRobotId() { return robotId; }

    /** {@code true} if this body was created with an alliance-station identity. */
    public boolean isRobot() { return robotId != null; }

    // Pose
    public Pose3d getPose() { return body.getPose(); }

    public Translation3d getPosition() {
        return new Translation3d(body.posX, body.posY, body.posZ);
    }

    public Rotation3d getRotation() {
        return body.getPose().getRotation();
    }

    /** Teleport the body to a new pose (zeros velocities are preserved). */
    public void setPose(Pose3d pose) { body.setPose(pose); }

    public void setPosition(double x, double y, double z) {
        body.posX = x; body.posY = y; body.posZ = z;
        body.refreshWorldInertia();
    }

    // Velocity
    public Translation3d getLinearVelocity() { return body.getLinearVelocity(); }
    public Translation3d getAngularVelocity() { return body.getAngularVelocity(); }

    public void setLinearVelocity(Translation3d v) { body.setLinearVelocity(v); }
    public void setAngularVelocity(Translation3d omega) { body.setAngularVelocity(omega); }

    public void setLinearVelocity(double vx, double vy, double vz) {
        body.velX = vx; body.velY = vy; body.velZ = vz;
    }

    public void setAngularVelocity(double omX, double omY, double omZ) {
        body.omX = omX; body.omY = omY; body.omZ = omZ;
    }

    public double getSpeed() {
        double vx = body.velX, vy = body.velY, vz = body.velZ;
        return Math.sqrt(vx * vx + vy * vy + vz * vz);
    }

    // Actuator (force/torque input from robot code)
    /**
     * The actuator belonging to this body.
     *
     * <p>Set forces/torques on it each robot loop iteration; they are picked up on
     * the next physics tick.
     */
    public ActuatorForce getActuator() { return actuator; }

    // Mass / material
    public double getMass() { return body.invMass > 0 ? 1.0 / body.invMass : Double.POSITIVE_INFINITY; }
    public Material getMaterial() { return body.material; }
    public void setMaterial(Material m) { body.material = m; }

    // Collider

    public ColliderShape getCollider() { return body.collider; }

    // State queries

    public boolean isStatic() { return body.isStatic(); }

    /** Return the 6-element world-space AABB [minX,minY,minZ,maxX,maxY,maxZ]. */
    public double[] getAABB() { return body.aabb.clone(); }

    @Override
    public String toString() {
        return "SimBody{id=" + body.id + ", name=" + body.name
            + ", pos=(" + body.posX + "," + body.posY + "," + body.posZ + ")}";
    }
}
