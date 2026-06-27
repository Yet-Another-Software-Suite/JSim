package jsim.core;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class Vec3Test {

    private static final double EPS = 1e-12;

    @Test
    void crossProduct_standardBasis() {
        double[] i = Vec3.of(1, 0, 0);
        double[] j = Vec3.of(0, 1, 0);
        double[] k = Vec3.cross(i, j);
        assertArrayEquals(new double[]{0, 0, 1}, k, EPS);
    }

    @Test
    void crossProduct_antiCommutative() {
        double[] a = Vec3.of(1, 2, 3);
        double[] b = Vec3.of(4, 5, 6);
        double[] axb = Vec3.cross(a, b);
        double[] bxa = Vec3.cross(b, a);
        assertArrayEquals(Vec3.negate(bxa), axb, EPS);
    }

    @Test
    void crossProduct_parallelVectorsAreZero() {
        double[] a = Vec3.of(1, 0, 0);
        double[] b = Vec3.of(3, 0, 0);
        double[] c = Vec3.cross(a, b);
        assertEquals(0, Vec3.length(c), EPS);
    }

    @Test
    void dotProduct_knownValue() {
        // (1,2,3)·(4,5,6) = 4+10+18 = 32
        double d = Vec3.dot(Vec3.of(1, 2, 3), Vec3.of(4, 5, 6));
        assertEquals(32.0, d, EPS);
    }

    @Test
    void dotProduct_orthogonalIsZero() {
        assertEquals(0, Vec3.dot(Vec3.of(1, 0, 0), Vec3.of(0, 1, 0)), EPS);
    }

    @Test
    void add() {
        double[] r = Vec3.add(Vec3.of(1, 2, 3), Vec3.of(4, 5, 6));
        assertArrayEquals(new double[]{5, 7, 9}, r, EPS);
    }

    @Test
    void sub() {
        double[] r = Vec3.sub(Vec3.of(5, 7, 9), Vec3.of(1, 2, 3));
        assertArrayEquals(new double[]{4, 5, 6}, r, EPS);
    }

    @Test
    void scale() {
        double[] r = Vec3.scale(Vec3.of(1, 2, 3), 2.0);
        assertArrayEquals(new double[]{2, 4, 6}, r, EPS);
    }

    @Test
    void length() {
        // 3-4-5 right triangle
        assertEquals(5.0, Vec3.length(Vec3.of(3, 4, 0)), EPS);
    }

    @Test
    void lengthSq() {
        assertEquals(14.0, Vec3.lengthSq(Vec3.of(1, 2, 3)), EPS);
    }

    @Test
    void normalize_unitVector() {
        double[] v = Vec3.of(3, 0, 4);
        double[] n = Vec3.normalize(v);
        assertEquals(1.0, Vec3.length(n), EPS);
        assertEquals(0.6, n[0], EPS);
        assertEquals(0.0, n[1], EPS);
        assertEquals(0.8, n[2], EPS);
    }

    @Test
    void normalize_zeroVectorDoesNotThrow() {
        double[] n = Vec3.normalize(Vec3.of(0, 0, 0));
        assertEquals(0, Vec3.length(n), EPS);
    }

    @Test
    void addScaled_mutatesFirstArg() {
        double[] a = new double[]{1, 2, 3};
        double[] b = Vec3.of(1, 1, 1);
        Vec3.addScaled(a, b, 2.0);
        assertArrayEquals(new double[]{3, 4, 5}, a, EPS);
    }

    @Test
    void negate() {
        double[] n = Vec3.negate(Vec3.of(1, -2, 3));
        assertArrayEquals(new double[]{-1, 2, -3}, n, EPS);
    }

    @Test
    void scalarTripleProduct_geometricIdentity() {
        // a·(b×c) == b·(c×a) == c·(a×b)
        double[] a = Vec3.of(1, 2, 3);
        double[] b = Vec3.of(4, 5, 6);
        double[] c = Vec3.of(7, 8, 10);
        double abc = Vec3.dot(a, Vec3.cross(b, c));
        double bca = Vec3.dot(b, Vec3.cross(c, a));
        double cab = Vec3.dot(c, Vec3.cross(a, b));
        assertEquals(abc, bca, EPS);
        assertEquals(bca, cab, EPS);
    }
}
