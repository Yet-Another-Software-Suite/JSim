// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.api;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Small 3x3 matrix helper used by the Fuel2026 example path.
 */
public class Matrix3 {
    /** Matrix element at row 0, column 0. */
    public final double m00;
    /** Matrix element at row 0, column 1. */
    public final double m01;
    /** Matrix element at row 0, column 2. */
    public final double m02;
    /** Matrix element at row 1, column 0. */
    public final double m10;
    /** Matrix element at row 1, column 1. */
    public final double m11;
    /** Matrix element at row 1, column 2. */
    public final double m12;
    /** Matrix element at row 2, column 0. */
    public final double m20;
    /** Matrix element at row 2, column 1. */
    public final double m21;
    /** Matrix element at row 2, column 2. */
    public final double m22;

    /**
     * Creates an identity matrix.
     */
    public Matrix3() {
        this(1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0);
    }

    /**
     * Creates a matrix from its nine elements.
     *
     * @param m00 element at row 0, column 0
     * @param m01 element at row 0, column 1
     * @param m02 element at row 0, column 2
     * @param m10 element at row 1, column 0
     * @param m11 element at row 1, column 1
     * @param m12 element at row 1, column 2
     * @param m20 element at row 2, column 0
     * @param m21 element at row 2, column 1
     * @param m22 element at row 2, column 2
     */
    public Matrix3(
            double m00, double m01, double m02,
            double m10, double m11, double m12,
            double m20, double m21, double m22) {
        this.m00 = m00;
        this.m01 = m01;
        this.m02 = m02;
        this.m10 = m10;
        this.m11 = m11;
        this.m12 = m12;
        this.m20 = m20;
        this.m21 = m21;
        this.m22 = m22;
    }

    /**
     * Returns the identity matrix.
     *
     * @return an identity matrix
     */
    public static Matrix3 identity() {
        return new Matrix3();
    }

    /**
     * Multiplies this matrix by a translation vector.
     *
     * @param v the vector to transform
     * @return the transformed vector
     */
    public Translation3d multiply(Translation3d v) {
        return new Translation3d(
                m00 * v.getX() + m01 * v.getY() + m02 * v.getZ(),
                m10 * v.getX() + m11 * v.getY() + m12 * v.getZ(),
                m20 * v.getX() + m21 * v.getY() + m22 * v.getZ());
    }
}
