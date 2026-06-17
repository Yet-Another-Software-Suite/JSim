package frc.jsim.api;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.jsim.collision.BoxCollider;
import frc.jsim.collision.ColliderShape;
import frc.jsim.collision.PlaneCollider;
import frc.jsim.collision.SphereCollider;
import frc.jsim.dynamics.RigidBody;
import frc.jsim.dynamics.RigidBodyFlags;
import frc.jsim.forces.ActuatorForce;
import frc.jsim.material.Material;

/**
 * Fluent builder for creating {@link SimBody} instances.
 *
 * <p>Example usage:
 * <pre>{@code
 * SimBody robot = world.addBody(new SimBodyBuilder("Robot")
 *     .pose(new Pose3d(1, 2, 0.1, new Rotation3d()))
 *     .mass(50)  // kg
 *     .boxCollider(0.5, 0.4, 0.15)  // half-extents in metres
 *     .material(Material.CARPET)
 *     .noGravity());
 * }</pre>
 */
public final class SimBodyBuilder {
    private final String name;
    private int flags = RigidBodyFlags.DYNAMIC;
    private Material material = Material.DEFAULT;
    private ColliderShape collider;
    private RobotId robotId = null;

    private double mass = 1.0;
    private double ixx = 1.0, iyy = 1.0, izz = 1.0;

    private double posX, posY, posZ;          // metres
    private double qW = 1, qX, qY, qZ;       // unit quaternion (w,x,y,z)

    private double velX, velY, velZ;          // m/s
    private double omX, omY, omZ;             // rad/s

    public SimBodyBuilder(String name) { this.name = name; }

    // Pose
    public SimBodyBuilder pose(Pose3d pose) {
        posX = pose.getX(); posY = pose.getY(); posZ = pose.getZ();
        Quaternion q = pose.getRotation().getQuaternion();
        qW = q.getW(); qX = q.getX(); qY = q.getY(); qZ = q.getZ();
        return this;
    }

    public SimBodyBuilder position(double x, double y, double z) {
        posX = x; posY = y; posZ = z; return this;
    }

    public SimBodyBuilder rotation(Rotation3d r) {
        Quaternion q = r.getQuaternion();
        qW = q.getW(); qX = q.getX(); qY = q.getY(); qZ = q.getZ();
        return this;
    }

    // Velocity

    public SimBodyBuilder linearVelocity(double vx, double vy, double vz) {
        velX = vx; velY = vy; velZ = vz; return this;
    }

    public SimBodyBuilder angularVelocity(double ox, double oy, double oz) {
        omX = ox; omY = oy; omZ = oz; return this;
    }

    // Mass / inertia

    public SimBodyBuilder mass(double kg) { this.mass = kg; return this; }

    /**
     * Set diagonal body-frame inertia tensor components (kg·m²).
     * If not called, the builder estimates inertia from the mass and collider shape.
     */
    public SimBodyBuilder inertia(double ixx, double iyy, double izz) {
        this.ixx = ixx; this.iyy = iyy; this.izz = izz; return this;
    }

    // Collider shortcuts

    public SimBodyBuilder sphereCollider(double radius) {
        collider = new SphereCollider(radius);
        return estimateInertiaIfNeeded(radius);
    }

    public SimBodyBuilder boxCollider(double halfX, double halfY, double halfZ) {
        collider = new BoxCollider(halfX, halfY, halfZ);
        return estimateInertiaIfNeeded(halfX, halfY, halfZ);
    }

    /** Infinite half-space floor/wall. Automatically sets STATIC flag. */
    public SimBodyBuilder planeCollider(double nx, double ny, double nz, double offset) {
        collider = new PlaneCollider(nx, ny, nz, offset);
        isStatic();
        return this;
    }

    public SimBodyBuilder collider(ColliderShape shape) {
        this.collider = shape; return this;
    }

    // Material

    public SimBodyBuilder material(Material m) { this.material = m; return this; }

    /**
     * Tag this body as an FRC robot at the given alliance station.
     * The body can then be retrieved from the world with
     * {@link SimWorld#findRobot(RobotId)}.
     */
    public SimBodyBuilder robotId(RobotId id) { this.robotId = id; return this; }

    // Flags

    public SimBodyBuilder isStatic() { flags |= RigidBodyFlags.STATIC; return this; }
    public SimBodyBuilder noGravity() { flags |= RigidBodyFlags.NO_GRAVITY; return this; }
    public SimBodyBuilder noCollision() { flags |= RigidBodyFlags.NO_COLLISION; return this; }
    public SimBodyBuilder fixedRotation() { flags |= RigidBodyFlags.FIXED_ROTATION; return this; }

    // Build (called internally by SimWorld.addBody)

    SimBody build() {
        RigidBody body = new RigidBody(name, flags, material);
        body.posX = posX; body.posY = posY; body.posZ = posZ;
        body.qW = qW; body.qX = qX; body.qY = qY; body.qZ = qZ;
        body.velX = velX; body.velY = velY; body.velZ = velZ;
        body.omX = omX; body.omY = omY; body.omZ = omZ;
        body.collider = collider;

        if (RigidBodyFlags.isSet(flags, RigidBodyFlags.STATIC)) {
            body.setStatic();
        } else {
            body.setMassProperties(mass, ixx, iyy, izz);
        }

        ActuatorForce actuator = new ActuatorForce(body);
        return new SimBody(body, actuator, robotId);
    }

    // Inertia estimation

    private SimBodyBuilder estimateInertiaIfNeeded(double radius) {
        // Solid sphere: I = 2/5 * m * r²
        double v = 0.4 * mass * radius * radius;
        ixx = iyy = izz = v;
        return this;
    }

    private SimBodyBuilder estimateInertiaIfNeeded(double hx, double hy, double hz) {
        // Solid box: I_xx = m/12 * (4hy² + 4hz²) with half-extents
        ixx = mass / 3.0 * (hy * hy + hz * hz);
        iyy = mass / 3.0 * (hx * hx + hz * hz);
        izz = mass / 3.0 * (hx * hx + hy * hy);
        return this;
    }
}
