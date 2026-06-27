package jsim.core;

import edu.wpi.first.math.geometry.Translation3d;

/** Internal 3-component vector math over raw doubles for deterministic physics. */
public final class Vec3 {
    private Vec3() {}

    /**
     * Returns a new vector {@code [x, y, z]}.
     *
     * @param x the x component
     * @param y the y component
     * @param z the z component
     * @return a new 3-element double array
     */
    public static double[] of(double x, double y, double z) {
        return new double[] {x, y, z};
    }

    /**
     * Converts a {@link Translation3d} to a raw double array.
     *
     * @param t the translation to convert
     * @return a new 3-element double array {@code [x, y, z]}
     */
    public static double[] fromTranslation(Translation3d t) {
        return of(t.getX(), t.getY(), t.getZ());
    }

    /**
     * Converts a raw double array to a {@link Translation3d}.
     *
     * @param v the 3-element vector to convert
     * @return a {@link Translation3d} with the same components
     */
    public static Translation3d toTranslation(double[] v) {
        return new Translation3d(v[0], v[1], v[2]);
    }

    /**
     * Returns the cross product {@code a × b}.
     *
     * @param a the left-hand vector
     * @param b the right-hand vector
     * @return a new vector representing {@code a × b}
     */
    public static double[] cross(double[] a, double[] b) {
        return of(
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]);
    }

    /**
     * Returns the dot product {@code a · b}.
     *
     * @param a the left-hand vector
     * @param b the right-hand vector
     * @return the scalar dot product
     */
    public static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    /**
     * Returns {@code a + b} as a new vector.
     *
     * @param a the first vector
     * @param b the second vector
     * @return a new vector representing the sum
     */
    public static double[] add(double[] a, double[] b) {
        return of(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
    }

    /**
     * Returns {@code a - b} as a new vector.
     *
     * @param a the vector to subtract from
     * @param b the vector to subtract
     * @return a new vector representing the difference
     */
    public static double[] sub(double[] a, double[] b) {
        return of(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    }

    /**
     * Returns {@code a * s} as a new vector.
     *
     * @param a the vector to scale
     * @param s the scalar multiplier
     * @return a new scaled vector
     */
    public static double[] scale(double[] a, double s) {
        return of(a[0] * s, a[1] * s, a[2] * s);
    }

    /**
     * Returns the squared length of {@code a}.
     *
     * @param a the vector
     * @return the squared Euclidean length
     */
    public static double lengthSq(double[] a) {
        return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    }

    /**
     * Returns the length of {@code a}.
     *
     * @param a the vector
     * @return the Euclidean length
     */
    public static double length(double[] a) {
        return Math.sqrt(lengthSq(a));
    }

    /**
     * Returns a unit vector in the direction of {@code a}, or zero if {@code a} is near-zero.
     *
     * @param a the vector to normalize
     * @return a new unit vector, or {@code [0, 0, 0]} if the input length is below 1e-12
     */
    public static double[] normalize(double[] a) {
        double len = length(a);
        if (len < 1e-12) return of(0, 0, 0);
        return scale(a, 1.0 / len);
    }

    /**
     * Adds {@code b * s} into {@code a} in-place.
     *
     * @param a the vector to accumulate into
     * @param b the vector to scale and add
     * @param s the scalar multiplier
     */
    public static void addScaled(double[] a, double[] b, double s) {
        a[0] += b[0] * s;
        a[1] += b[1] * s;
        a[2] += b[2] * s;
    }

    /**
     * Returns {@code -a} as a new vector.
     *
     * @param a the vector to negate
     * @return a new vector with all components negated
     */
    public static double[] negate(double[] a) {
        return of(-a[0], -a[1], -a[2]);
    }
}
