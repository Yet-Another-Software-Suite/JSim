// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.api;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Represents a 2026 Fuel game piece.
 * Integrates vector, quaternion, and matrix math for simulation.
 *
 * <p>Usage example:
 * <pre>{@code
 * Fuel2026 fuel = new Fuel2026();
 * // Simulate a shot with a 0.3s time-of-flight and 30° exit angle:
 * fuel.shoot(
 *     new Translation3d(0.0, 0.0, 0.9),          // exit point relative to robot
 *     0.3,                                          // seconds to target
 *     new Rotation3d(0, Math.toRadians(30), 0)    // pitch angle
 * );
 * }</pre>
 */
public class Fuel2026 extends GamePieceState {
    /**
     * Constructs a Fuel2026 game piece.
     */
    public Fuel2026() { super(GamePieceType.FUEL); }

    /**
     * Simulates shooting the fuel game piece using vector and quaternion math.
     * @param relativeStart The relative start position.
     * @param timeOfFlight The time of flight.
     * @param exitAngle The exit angle (as Rotation3d, converted to Quaternion).
     */
    public void shoot(Translation3d relativeStart, double timeOfFlight, Rotation3d exitAngle) {
        setExitAngle(exitAngle);

        // Example: rotate a velocity vector by the exit angle
        Translation3d velocity = new Translation3d(1, 0, 0);
        Translation3d rotatedVelocity = rotateVectorByRotation3d(exitAngle, velocity);

        // Example: use a matrix for further transformation (identity for now)
        Matrix3 m = Matrix3.identity();
        m.multiply(rotatedVelocity);
    }

    private static Translation3d rotateVectorByRotation3d(Rotation3d rotation, Translation3d vector) {
        double[] quaternion = quaternionFromRotation3d(rotation);
        double[] vectorQuaternion = multiplyQuaternion(0.0, vector.getX(), vector.getY(), vector.getZ(),
                quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        double[] rotatedQuaternion = multiplyQuaternion(vectorQuaternion[0], vectorQuaternion[1], vectorQuaternion[2], vectorQuaternion[3],
                quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]);
        return new Translation3d(rotatedQuaternion[1], rotatedQuaternion[2], rotatedQuaternion[3]);
    }

    private static double[] quaternionFromRotation3d(Rotation3d r) {
        double cy = Math.cos(r.getZ() * 0.5);
        double sy = Math.sin(r.getZ() * 0.5);
        double cp = Math.cos(r.getY() * 0.5);
        double sp = Math.sin(r.getY() * 0.5);
        double cr = Math.cos(r.getX() * 0.5);
        double sr = Math.sin(r.getX() * 0.5);

        double w = cr * cp * cy + sr * sp * sy;
        double x = sr * cp * cy - cr * sp * sy;
        double y = cr * sp * cy + sr * cp * sy;
        double z = cr * cp * sy - sr * sp * cy;
        return new double[] {w, x, y, z};
    }

    private static double[] multiplyQuaternion(
            double aw,
            double ax,
            double ay,
            double az,
            double bw,
            double bx,
            double by,
            double bz) {
        return new double[] {
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw
        };
    }
}
