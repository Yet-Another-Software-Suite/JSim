package jsim.core;

/**
 * Internal row-major 3x3 matrix math for deterministic physics.
 *
 * <p>Index layout: [0 1 2 / 3 4 5 / 6 7 8] where row r, col c = index r*3+c.
 */
public final class Mat3 {
    private Mat3() {}

    /**
     * Returns the 3x3 identity matrix as a 9-element row-major array.
     *
     * @return identity matrix
     */
    public static double[] identity() {
        return new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1};
    }

    /**
     * Returns a diagonal 3x3 matrix with the given diagonal elements.
     *
     * @param a first diagonal element
     * @param b second diagonal element
     * @param c third diagonal element
     * @return 9-element row-major diagonal matrix
     */
    public static double[] diagonal(double a, double b, double c) {
        return new double[] {a, 0, 0, 0, b, 0, 0, 0, c};
    }

    /**
     * Build rotation matrix from unit quaternion (w, x, y, z).
     *
     * @param qw quaternion W component
     * @param qx quaternion X component
     * @param qy quaternion Y component
     * @param qz quaternion Z component
     * @return 3x3 row-major rotation matrix as a 9-element array
     */
    public static double[] fromQuaternion(double qw, double qx, double qy, double qz) {
        double x2 = 2 * qx * qx, y2 = 2 * qy * qy, z2 = 2 * qz * qz;
        double xy = 2 * qx * qy, xz = 2 * qx * qz, yz = 2 * qy * qz;
        double wx = 2 * qw * qx, wy = 2 * qw * qy, wz = 2 * qw * qz;
        return new double[] {
            1 - y2 - z2, xy - wz,     xz + wy,
            xy + wz,     1 - x2 - z2, yz - wx,
            xz - wy,     yz + wx,     1 - x2 - y2
        };
    }

    /**
     * Multiply a 3x3 matrix by a 3-vector (m * v).
     *
     * @param m 3x3 row-major matrix (9 elements)
     * @param v input vector (3 elements)
     * @return result vector (3 elements)
     */
    public static double[] mulVec(double[] m, double[] v) {
        return new double[] {
            m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2]
        };
    }

    /**
     * Multiply two 3x3 row-major matrices (a * b).
     *
     * @param a left matrix (9 elements)
     * @param b right matrix (9 elements)
     * @return product matrix (9 elements)
     */
    public static double[] mul(double[] a, double[] b) {
        double[] r = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
            }
        }
        return r;
    }

    /**
     * Transpose of a 3x3 row-major matrix.
     *
     * @param m input matrix (9 elements)
     * @return transposed matrix (9 elements)
     */
    public static double[] transpose(double[] m) {
        return new double[] {
            m[0], m[3], m[6],
            m[1], m[4], m[7],
            m[2], m[5], m[8]
        };
    }

    /**
     * Compute R * diag(d0,d1,d2) * R^T efficiently.
     * Used to transform body-frame diagonal inertia to world frame.
     *
     * @param R  rotation matrix (9 elements, row-major)
     * @param d0 first diagonal element
     * @param d1 second diagonal element
     * @param d2 third diagonal element
     * @return 3x3 result matrix (9 elements)
     */
    public static double[] rotateInertia(double[] R, double d0, double d1, double d2) {
        // tmp = R * diag
        double[] tmp = new double[9];
        for (int i = 0; i < 3; i++) {
            tmp[i * 3]     = R[i * 3]     * d0;
            tmp[i * 3 + 1] = R[i * 3 + 1] * d1;
            tmp[i * 3 + 2] = R[i * 3 + 2] * d2;
        }
        // result = tmp * R^T = tmp * R^T
        return mul(tmp, transpose(R));
    }

    /**
     * Invert a 3x3 matrix using Cramer's rule.
     * Returns the identity matrix if the input is singular.
     *
     * @param m input matrix (9 elements, row-major)
     * @return inverted matrix (9 elements)
     */
    public static double[] invert(double[] m) {
        double a = m[0], b = m[1], c = m[2];
        double d = m[3], e = m[4], f = m[5];
        double g = m[6], h = m[7], k = m[8];

        double A = e * k - f * h;
        double B = -(d * k - f * g);
        double C = d * h - e * g;
        double det = a * A + b * B + c * C;
        if (Math.abs(det) < 1e-18) return identity();
        double inv = 1.0 / det;
        return new double[] {
            A * inv,               -(b * k - c * h) * inv,  (b * f - c * e) * inv,
            B * inv,               (a * k - c * g) * inv,  -(a * f - c * d) * inv,
            C * inv,              -(a * h - b * g) * inv,   (a * e - b * d) * inv
        };
    }
}
