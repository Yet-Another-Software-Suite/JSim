package frc.jsim.dynamics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.jsim.collision.ColliderShape;
import frc.jsim.core.Mat3;
import frc.jsim.core.Vec3;
import frc.jsim.material.Material;

/**
 * A single rigid body in the simulation.
 *
 * <p>All physics state is stored as raw doubles for determinism and performance.
 * WPILib types are used only at the read/write surface (getPose, setPose, etc.).
 *
 * <p>Bodies are created and owned by {@link frc.jsim.core.PhysicsWorld}; use
 * {@link frc.jsim.api.SimBodyBuilder} to configure them before adding.
 */
public final class RigidBody {
    // ---- Identity -------------------------------------------------------
    /** World-assigned unique ID, set by PhysicsWorld on registration. */
    public int id = -1;

    /** Human-readable label for debugging. */
    public final String name;

    /** Bitmask of {@link RigidBodyFlags}. */
    public int flags;

    // ---- Pose (position + orientation) ----------------------------------
    public double posX, posY, posZ;
    /** Unit quaternion orientation (w + xi + yj + zk). */
    public double qW = 1, qX, qY, qZ;

    // ---- Velocities -----------------------------------------------------
    public double velX, velY, velZ;
    /** Angular velocity in world frame (rad/s). */
    public double omX, omY, omZ;

    // ---- Accumulators (cleared each tick) -------------------------------
    public double forceX, forceY, forceZ;
    public double torqueX, torqueY, torqueZ;

    // ---- Mass properties ------------------------------------------------
    /** 1/mass. 0 for static bodies (infinite mass). */
    public double invMass;
    /** Body-frame diagonal inertia tensor components (kg·m²). */
    public double iBodyXX, iBodyYY, iBodyZZ;
    /** World-frame inverse inertia tensor (3x3 row-major). Recomputed each tick. */
    public final double[] invIWorld = new double[9];

    // ---- Material + collider --------------------------------------------
    public Material material;
    public ColliderShape collider;

    // ---- Derived state (recomputed by PhysicsWorld.updateDerivedState) --
    /** Cached world-frame AABB for broadphase. Stored as [minX,minY,minZ,maxX,maxY,maxZ]. */
    public final double[] aabb = new double[6];

    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    public RigidBody(String name, int flags, Material material) {
        this.name = name;
        this.flags = flags;
        this.material = material;
    }

    // -----------------------------------------------------------------------
    // WPILib surface
    // -----------------------------------------------------------------------

    public Pose3d getPose() {
        return new Pose3d(
            new Translation3d(posX, posY, posZ),
            new Rotation3d(new Quaternion(qW, qX, qY, qZ)));
    }

    public void setPose(Pose3d pose) {
        posX = pose.getX();
        posY = pose.getY();
        posZ = pose.getZ();
        Quaternion q = pose.getRotation().getQuaternion();
        qW = q.getW(); qX = q.getX(); qY = q.getY(); qZ = q.getZ();
        normalizeQuaternion();
        refreshWorldInertia();
    }

    public Translation3d getLinearVelocity() {
        return new Translation3d(velX, velY, velZ);
    }

    public void setLinearVelocity(Translation3d v) {
        velX = v.getX(); velY = v.getY(); velZ = v.getZ();
    }

    public Translation3d getAngularVelocity() {
        return new Translation3d(omX, omY, omZ);
    }

    public void setAngularVelocity(Translation3d omega) {
        omX = omega.getX(); omY = omega.getY(); omZ = omega.getZ();
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /** Apply a force (world frame, Newtons) at the centre of mass. */
    public void applyForce(double fx, double fy, double fz) {
        forceX += fx; forceY += fy; forceZ += fz;
    }

    /** Apply a torque (world frame, N·m). */
    public void applyTorque(double tx, double ty, double tz) {
        torqueX += tx; torqueY += ty; torqueZ += tz;
    }

    /**
     * Apply a world-frame force at a world-frame point.
     * Generates both a linear force and a torque about the centre of mass.
     */
    public void applyForceAtPoint(double fx, double fy, double fz,
                                   double px, double py, double pz) {
        applyForce(fx, fy, fz);
        double rx = px - posX, ry = py - posY, rz = pz - posZ;
        double[] torque = Vec3.cross(new double[]{rx, ry, rz}, new double[]{fx, fy, fz});
        applyTorque(torque[0], torque[1], torque[2]);
    }

    /** Zero out force and torque accumulators. Called at the start of every tick. */
    public void clearAccumulators() {
        forceX = forceY = forceZ = 0;
        torqueX = torqueY = torqueZ = 0;
    }

    /** Set mass (kg) and a diagonal body-frame inertia tensor (kg·m²). */
    public void setMassProperties(double mass, double ixx, double iyy, double izz) {
        invMass = (mass <= 0) ? 0 : 1.0 / mass;
        iBodyXX = ixx; iBodyYY = iyy; iBodyZZ = izz;
        refreshWorldInertia();
    }

    /** Make this body infinitely massive (static boundary). */
    public void setStatic() {
        invMass = 0;
        iBodyXX = iBodyYY = iBodyZZ = 0;
        java.util.Arrays.fill(invIWorld, 0);
        flags |= RigidBodyFlags.STATIC;
    }

    /** Recompute invIWorld from current orientation and body-frame inertia. */
    public void refreshWorldInertia() {
        if (RigidBodyFlags.isSet(flags, RigidBodyFlags.STATIC)
                || RigidBodyFlags.isSet(flags, RigidBodyFlags.FIXED_ROTATION)) {
            java.util.Arrays.fill(invIWorld, 0);
            return;
        }
        double[] R = Mat3.fromQuaternion(qW, qX, qY, qZ);
        // World-frame inertia: I_w = R * I_b * R^T
        // World-frame inverse: I_w^-1 = R * diag(1/Ixx, 1/Iyy, 1/Izz) * R^T
        double invXX = (iBodyXX > 0) ? 1.0 / iBodyXX : 0;
        double invYY = (iBodyYY > 0) ? 1.0 / iBodyYY : 0;
        double invZZ = (iBodyZZ > 0) ? 1.0 / iBodyZZ : 0;
        double[] iw = Mat3.rotateInertia(R, invXX, invYY, invZZ);
        System.arraycopy(iw, 0, invIWorld, 0, 9);
    }

    /** Re-normalize the orientation quaternion (guards against drift). */
    public void normalizeQuaternion() {
        double len = Math.sqrt(qW * qW + qX * qX + qY * qY + qZ * qZ);
        if (len < 1e-12) { qW = 1; qX = qY = qZ = 0; return; }
        qW /= len; qX /= len; qY /= len; qZ /= len;
    }

    /** Whether this body participates as an infinite-mass static wall. */
    public boolean isStatic() {
        return RigidBodyFlags.isSet(flags, RigidBodyFlags.STATIC);
    }

    public int getId() { return id; }
}
