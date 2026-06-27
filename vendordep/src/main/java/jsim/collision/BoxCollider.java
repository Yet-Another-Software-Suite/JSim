package jsim.collision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import jsim.core.Mat3;

/**
 * Axis-aligned-in-body-space box centred at the body's local origin.
 * Half-extents are the distances from the centre to each face.
 */
public final class BoxCollider extends ColliderShape {
    /** Half-extents along each body-local axis (metres). */
    public final double halfX, halfY, halfZ;

    /**
     * Create a box collider with the given half-extents.
     *
     * @param halfX half-extent along the body-local X axis (metres)
     * @param halfY half-extent along the body-local Y axis (metres)
     * @param halfZ half-extent along the body-local Z axis (metres)
     */
    public BoxCollider(double halfX, double halfY, double halfZ) {
        this.halfX = halfX; this.halfY = halfY; this.halfZ = halfZ;
    }

    @Override
    public Type getType() { return Type.BOX; }

    @Override
    public void computeAABB(Pose3d pose, double[] out) {
        double posX = pose.getX(), posY = pose.getY(), posZ = pose.getZ();
        Quaternion q = pose.getRotation().getQuaternion();
        // Rotate half-extents using the absolute value of each column of R.
        double[] R = Mat3.fromQuaternion(q.getW(), q.getX(), q.getY(), q.getZ());
        // World-space half-extents of the OBB's AABB:
        // h_world_i = |R[i,0]|*hX + |R[i,1]|*hY + |R[i,2]|*hZ
        double wx = Math.abs(R[0]) * halfX + Math.abs(R[1]) * halfY + Math.abs(R[2]) * halfZ;
        double wy = Math.abs(R[3]) * halfX + Math.abs(R[4]) * halfY + Math.abs(R[5]) * halfZ;
        double wz = Math.abs(R[6]) * halfX + Math.abs(R[7]) * halfY + Math.abs(R[8]) * halfZ;
        out[0] = posX - wx; out[1] = posY - wy; out[2] = posZ - wz;
        out[3] = posX + wx; out[4] = posY + wy; out[5] = posZ + wz;
    }
}
