package frc.jsim.api;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
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

    /** @return world-assigned unique ID of the underlying physics body */
    public int getId() { return body.id; }
    /** @return the name assigned to this body at creation time */
    public String getName() { return body.name; }

    /**
     * The FRC alliance-station identity of this body, or {@code null} if it
     * was not registered with {@link SimBodyBuilder#robotId(RobotId)}.
     *
     * @return robot identity, or {@code null}
     */
    public RobotId getRobotId() { return robotId; }

    /**
     * {@code true} if this body was created with an alliance-station identity.
     *
     * @return {@code true} if a robot ID is set
     */
    public boolean isRobot() { return robotId != null; }

    // Pose
    /** @return current world-frame pose */
    public Pose3d getPose() { return body.getPose(); }

    /** @return current position in world frame (metres) */
    public Translation3d getPosition() {
        return new Translation3d(body.posX, body.posY, body.posZ);
    }

    /** @return current orientation in world frame */
    public Rotation3d getRotation() {
        return body.getPose().getRotation();
    }

    /**
     * Teleport the body to a new pose (velocities are preserved).
     *
     * @param pose new world-frame pose
     */
    public void setPose(Pose3d pose) { body.setPose(pose); }

    public void setPosition(double x, double y, double z) {
        body.posX = x; body.posY = y; body.posZ = z;
        body.refreshWorldInertia();
    }

    // Velocity
    /** @return linear velocity in world frame (m/s) */
    public Translation3d getLinearVelocity() { return body.getLinearVelocity(); }
    /** @return angular velocity in world frame (rad/s) */
    public Translation3d getAngularVelocity() { return body.getAngularVelocity(); }

    public void setLinearVelocity(Translation3d v) { body.setLinearVelocity(v); }
    public void setAngularVelocity(Translation3d omega) { body.setAngularVelocity(omega); }
    public void setLinearVelocity(double vx, double vy, double vz) {
        body.velX = vx; body.velY = vy; body.velZ = vz;
    }

    /**
     * @param omX angular velocity X component (rad/s)
     * @param omY angular velocity Y component (rad/s)
     * @param omZ angular velocity Z component (rad/s)
     */
    public void setAngularVelocity(double omX, double omY, double omZ) {
        body.omX = omX; body.omY = omY; body.omZ = omZ;
    }

    /** @return scalar speed (magnitude of linear velocity) */
    public LinearVelocity getSpeed() {
        double vx = body.velX, vy = body.velY, vz = body.velZ;
        return MetersPerSecond.of(Math.sqrt(vx * vx + vy * vy + vz * vz));
    }
    // Actuator (force/torque input from robot code)
    /**
     * The actuator belonging to this body.
     *
     * <p>Set forces/torques on it each robot loop iteration; they are picked up on
     * the next physics tick.
     *
     * @return this body's actuator force generator
     */
    public ActuatorForce getActuator() { return actuator; }

    // Mass / material
    /** @return mass of this body ({@code POSITIVE_INFINITY} for static bodies) */
    public Mass getMass() { return Kilograms.of(body.invMass > 0 ? 1.0 / body.invMass : Double.POSITIVE_INFINITY); }
    /** @return current surface material */
    public Material getMaterial() { return body.material; }
    public void setMaterial(Material m) { body.material = m; }

    // Collider

    /** @return the collision shape attached to this body, or {@code null} */
    public ColliderShape getCollider() { return body.collider; }

    // State queries

    /** @return {@code true} if this body has infinite mass and does not move */
    public boolean isStatic() { return body.isStatic(); }

    /**
     * Return the 6-element world-space AABB [minX,minY,minZ,maxX,maxY,maxZ] in metres.
     *
     * @return copy of the AABB array
     */
    public double[] getAABB() { return body.aabb.clone(); }

    @Override
    public String toString() {
        return "SimBody{id=" + body.id + ", name=" + body.name
            + ", pos=(" + body.posX + "," + body.posY + "," + body.posZ + ")}";
    }
}
