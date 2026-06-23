package frc.jsim.api;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Mass;
import static edu.wpi.first.units.Units.Kilograms;
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
 *     .mass(Kilograms.of(50))
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

    private double mass = 1.0;          // kg
    private double ixx = 1.0, iyy = 1.0, izz = 1.0;  // kg·m²

    private double posX, posY, posZ;          // metres
    private double qW = 1, qX, qY, qZ;       // unit quaternion (w,x,y,z)

    private double velX, velY, velZ;          // m/s
    private double omX, omY, omZ;             // rad/s

    /**
     * Creates a builder for a body with the given name.
     *
     * @param name human-readable label for the body
     */
    public SimBodyBuilder(String name) { this.name = name; }

    // Pose

    /**
     * Sets the initial pose (position + rotation) of the body.
     *
     * @param pose the initial pose in world space
     * @return this builder
     */
    public SimBodyBuilder pose(Pose3d pose) {
        posX = pose.getX(); posY = pose.getY(); posZ = pose.getZ();
        Quaternion q = pose.getRotation().getQuaternion();
        qW = q.getW(); qX = q.getX(); qY = q.getY(); qZ = q.getZ();
        return this;
    }

    /**
     * Sets the initial position of the body in world space.
     *
     * @param x position along the X axis (metres)
     * @param y position along the Y axis (metres)
     * @param z position along the Z axis (metres)
     * @return this builder
     */
    public SimBodyBuilder position(double x, double y, double z) {
        posX = x; posY = y; posZ = z; return this;
    }

    /**
     * Sets the initial rotation of the body.
     *
     * @param r the initial orientation in world space
     * @return this builder
     */
    public SimBodyBuilder rotation(Rotation3d r) {
        Quaternion q = r.getQuaternion();
        qW = q.getW(); qX = q.getX(); qY = q.getY(); qZ = q.getZ();
        return this;
    }

    // Velocity

    /**
     * Sets the initial linear velocity of the body.
     *
     * @param vx velocity along the X axis (m/s)
     * @param vy velocity along the Y axis (m/s)
     * @param vz velocity along the Z axis (m/s)
     * @return this builder
     */
    public SimBodyBuilder linearVelocity(double vx, double vy, double vz) {
        velX = vx; velY = vy; velZ = vz; return this;
    }

    /**
     * Sets the initial angular velocity of the body.
     *
     * @param ox angular velocity about the X axis (rad/s)
     * @param oy angular velocity about the Y axis (rad/s)
     * @param oz angular velocity about the Z axis (rad/s)
     * @return this builder
     */
    public SimBodyBuilder angularVelocity(double ox, double oy, double oz) {
        omX = ox; omY = oy; omZ = oz; return this;
    }

    // Mass / inertia

    /**
     * Sets the mass of the body in kilograms.
     *
     * @param kg mass in kilograms (must be positive)
     * @return this builder
     */
    public SimBodyBuilder mass(double kg) { this.mass = kg; return this; }

    /**
     * Sets the mass of the body from a WPILib {@link Mass} measure.
     *
     * @param mass the body mass
     * @return this builder
     */
    public SimBodyBuilder mass(Mass mass) { this.mass = mass.in(Kilograms); return this; }

    /**
     * Set diagonal body-frame inertia tensor components (kg·m²).
     * If not called, the builder estimates inertia from the mass and collider shape.
     *
     * @param ixx moment of inertia about the body X axis (kg·m²)
     * @param iyy moment of inertia about the body Y axis (kg·m²)
     * @param izz moment of inertia about the body Z axis (kg·m²)
     * @return this builder
     */
    public SimBodyBuilder inertia(double ixx, double iyy, double izz) {
        this.ixx = ixx; this.iyy = iyy; this.izz = izz; return this;
    }

    // Collider shortcuts

    /**
     * Attaches a sphere collider and estimates inertia from mass and radius.
     *
     * @param radius sphere radius in metres; must be positive
     * @return this builder
     */
    public SimBodyBuilder sphereCollider(double radius) {
        collider = new SphereCollider(radius);
        return estimateInertiaIfNeeded(radius);
    }

    /**
     * Attaches a box collider and estimates inertia from mass and half-extents.
     *
     * @param halfX half-width along the X axis (metres)
     * @param halfY half-width along the Y axis (metres)
     * @param halfZ half-width along the Z axis (metres)
     * @return this builder
     */
    public SimBodyBuilder boxCollider(double halfX, double halfY, double halfZ) {
        collider = new BoxCollider(halfX, halfY, halfZ);
        return estimateInertiaIfNeeded(halfX, halfY, halfZ);
    }

    /**
     * Infinite half-space floor/wall. Automatically sets STATIC flag.
     *
     * @param nx     normal X component (will be normalized)
     * @param ny     normal Y component
     * @param nz     normal Z component
     * @param offset signed distance from the body origin to the plane surface along the normal
     * @return this builder
     */
    public SimBodyBuilder planeCollider(double nx, double ny, double nz, double offset) {
        collider = new PlaneCollider(nx, ny, nz, offset);
        isStatic();
        return this;
    }

    /**
     * Attaches a custom collider shape.
     *
     * @param shape the collider to attach
     * @return this builder
     */
    public SimBodyBuilder collider(ColliderShape shape) {
        this.collider = shape; return this;
    }

    // Material

    /**
     * Sets the surface material used for contact response.
     *
     * @param m the surface material
     * @return this builder
     */
    public SimBodyBuilder material(Material m) { this.material = m; return this; }

    /**
     * Tag this body as an FRC robot at the given alliance station.
     * The body can then be retrieved from the world with
     * {@link SimWorld#findRobot(RobotId)}.
     *
     * @param id the alliance-station identity to assign
     * @return this builder
     */
    public SimBodyBuilder robotId(RobotId id) { this.robotId = id; return this; }

    // Flags

    /**
     * Marks the body as static (infinite mass, never moved by the solver).
     *
     * @return this builder
     */
    public SimBodyBuilder isStatic() { flags |= RigidBodyFlags.STATIC; return this; }

    /**
     * Disables gravity for this body.
     *
     * @return this builder
     */
    public SimBodyBuilder noGravity() { flags |= RigidBodyFlags.NO_GRAVITY; return this; }

    /**
     * Disables collision detection for this body.
     *
     * @return this builder
     */
    public SimBodyBuilder noCollision() { flags |= RigidBodyFlags.NO_COLLISION; return this; }

    /**
     * Prevents rotational integration, keeping orientation constant.
     *
     * @return this builder
     */
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
