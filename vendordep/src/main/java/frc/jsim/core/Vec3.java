package frc.jsim.core;

import edu.wpi.first.math.geometry.Translation3d;

/** Internal 3-component vector math over raw doubles for deterministic physics. */
public final class Vec3 {
    private Vec3() {}

    public static double[] of(double x, double y, double z) {
        return new double[] {x, y, z};
    }

    public static double[] fromTranslation(Translation3d t) {
        return of(t.getX(), t.getY(), t.getZ());
    }

    public static Translation3d toTranslation(double[] v) {
        return new Translation3d(v[0], v[1], v[2]);
    }

    public static double[] cross(double[] a, double[] b) {
        return of(
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]);
    }

    public static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    public static double[] add(double[] a, double[] b) {
        return of(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
    }

    public static double[] sub(double[] a, double[] b) {
        return of(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    }

    public static double[] scale(double[] a, double s) {
        return of(a[0] * s, a[1] * s, a[2] * s);
    }

    public static double lengthSq(double[] a) {
        return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    }

    public static double length(double[] a) {
        return Math.sqrt(lengthSq(a));
    }

    public static double[] normalize(double[] a) {
        double len = length(a);
        if (len < 1e-12) return of(0, 0, 0);
        return scale(a, 1.0 / len);
    }

    /** Adds b*s into a in-place. */
    public static void addScaled(double[] a, double[] b, double s) {
        a[0] += b[0] * s;
        a[1] += b[1] * s;
        a[2] += b[2] * s;
    }

    public static double[] negate(double[] a) {
        return of(-a[0], -a[1], -a[2]);
    }
}
