package jsim.core;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class Mat3Test {

    private static final double EPS = 1e-10;

    // Helper: compare two 3x3 row-major matrices element-wise
    private static void assertMatEquals(double[] expected, double[] actual, double tol) {
        assertEquals(9, actual.length);
        for (int i = 0; i < 9; i++) {
            assertEquals(expected[i], actual[i], tol,
                "element [" + (i / 3) + "][" + (i % 3) + "]");
        }
    }

    @Test
    void identity_isMultiplicativeIdentity() {
        double[] I = Mat3.identity();
        double[] v = Vec3.of(1, 2, 3);
        assertArrayEquals(v, Mat3.mulVec(I, v), EPS);
    }

    @Test
    void diagonal_hasCorrectDiagonal() {
        double[] d = Mat3.diagonal(2, 3, 5);
        assertArrayEquals(Vec3.of(2, 0, 0), Mat3.mulVec(d, Vec3.of(1, 0, 0)), EPS);
        assertArrayEquals(Vec3.of(0, 3, 0), Mat3.mulVec(d, Vec3.of(0, 1, 0)), EPS);
        assertArrayEquals(Vec3.of(0, 0, 5), Mat3.mulVec(d, Vec3.of(0, 0, 1)), EPS);
    }

    @Test
    void fromQuaternion_identity_givesIdentityMatrix() {
        // Identity quaternion: (w=1, x=0, y=0, z=0)
        double[] R = Mat3.fromQuaternion(1, 0, 0, 0);
        assertMatEquals(Mat3.identity(), R, EPS);
    }

    @Test
    void fromQuaternion_90degAroundZ_mapsXtoY() {
        // 90° rotation around Z: q = (cos45°, 0, 0, sin45°)
        double s = Math.sqrt(0.5);
        double[] R = Mat3.fromQuaternion(s, 0, 0, s);
        // R * (1,0,0) should give (0,1,0)
        double[] result = Mat3.mulVec(R, Vec3.of(1, 0, 0));
        assertArrayEquals(Vec3.of(0, 1, 0), result, 1e-10);
    }

    @Test
    void fromQuaternion_180degAroundX_negatesYZ() {
        // 180° around X: q = (0, 1, 0, 0)
        double[] R = Mat3.fromQuaternion(0, 1, 0, 0);
        // R * (0,1,0) = (0,-1,0)
        assertArrayEquals(Vec3.of(0, -1, 0), Mat3.mulVec(R, Vec3.of(0, 1, 0)), EPS);
        // R * (0,0,1) = (0,0,-1)
        assertArrayEquals(Vec3.of(0, 0, -1), Mat3.mulVec(R, Vec3.of(0, 0, 1)), EPS);
        // R * (1,0,0) = (1,0,0) — X unchanged
        assertArrayEquals(Vec3.of(1, 0, 0), Mat3.mulVec(R, Vec3.of(1, 0, 0)), EPS);
    }

    @Test
    void mul_identityTimesMatrixIsMatrix() {
        double[] m = new double[]{1, 2, 3, 4, 5, 6, 7, 8, 9};
        assertMatEquals(m, Mat3.mul(Mat3.identity(), m), EPS);
        assertMatEquals(m, Mat3.mul(m, Mat3.identity()), EPS);
    }

    @Test
    void transpose_swapsRowsAndCols() {
        double[] m = new double[]{1, 2, 3, 4, 5, 6, 7, 8, 9};
        double[] t = Mat3.transpose(m);
        double[] expected = {1, 4, 7, 2, 5, 8, 3, 6, 9};
        assertMatEquals(expected, t, EPS);
    }

    @Test
    void transpose_ofTransposeIsOriginal() {
        double[] m = new double[]{1, 2, 3, 4, 5, 6, 7, 8, 9};
        assertMatEquals(m, Mat3.transpose(Mat3.transpose(m)), EPS);
    }

    @Test
    void rotateInertia_withIdentityRotation_preservesDiagonal() {
        // R = I, diag = (2,3,5) → result should be diag(2,3,5)
        double[] I = Mat3.identity();
        double[] result = Mat3.rotateInertia(I, 2, 3, 5);
        assertMatEquals(Mat3.diagonal(2, 3, 5), result, EPS);
    }

    @Test
    void rotateInertia_sphereIsIsotropicUnderRotation() {
        // For a sphere (Ixx=Iyy=Izz=c), any rotation gives the same diagonal
        double c = 0.5;
        double s45 = Math.sqrt(0.5);
        double[] R = Mat3.fromQuaternion(s45, 0, 0, s45); // 90° around Z
        double[] result = Mat3.rotateInertia(R, c, c, c);
        assertMatEquals(Mat3.diagonal(c, c, c), result, EPS);
    }

    @Test
    void invert_timesOriginalIsIdentity() {
        double[] m = {4, 7, 2, 1, 3, 1, 2, 5, 3};
        double[] inv = Mat3.invert(m);
        double[] product = Mat3.mul(m, inv);
        assertMatEquals(Mat3.identity(), product, 1e-10);
    }

    @Test
    void invert_identityInverseIsIdentity() {
        assertMatEquals(Mat3.identity(), Mat3.invert(Mat3.identity()), EPS);
    }

    @Test
    void fromQuaternion_isOrthogonal_detIsOne() {
        // Rotation matrices must satisfy R^T R = I
        double s = Math.sqrt(0.5);
        double[] R = Mat3.fromQuaternion(s, s * 0.5, s * 0.5, 0);
        // Normalize the quaternion first
        double qw = s, qx = s * 0.5, qy = s * 0.5, qz = 0.0;
        double len = Math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        R = Mat3.fromQuaternion(qw/len, qx/len, qy/len, qz/len);

        double[] Rt = Mat3.transpose(R);
        double[] product = Mat3.mul(Rt, R);
        assertMatEquals(Mat3.identity(), product, 1e-10);
    }
}
