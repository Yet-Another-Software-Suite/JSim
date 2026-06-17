package frc.jsim.core;

/** Simulation-wide constants. All values are SI (metres, kilograms, seconds). */
public final class SimConstants {
    private SimConstants() {}

    /** Default fixed timestep in seconds (20 ms = 50 Hz, matching FRC robot loop). */
    public static final double DEFAULT_DT = 0.02;

    /** Default gravity vector X component (m/s²); zero for FRC field convention. */
    public static final double GRAVITY_X = 0.0;
    /** Default gravity vector Y component (m/s²); zero for FRC field convention. */
    public static final double GRAVITY_Y = 0.0;
    /** Default gravity vector Z component (m/s²); negative Z = downward, FRC field convention. */
    public static final double GRAVITY_Z = -9.80665;

    /** Impulse solver iteration count per tick. Higher = more accurate but slower. */
    public static final int DEFAULT_SOLVER_ITERATIONS = 10;

    /**
     * Baumgarte positional correction factor.
     * Fraction of penetration depth corrected per tick.
     */
    public static final double BAUMGARTE_BETA = 0.2;

    /** Minimum penetration before positional correction is applied (slop). */
    public static final double PENETRATION_SLOP = 0.001;

    /** Relative velocity below this is treated as resting contact (no restitution). */
    public static final double RESTITUTION_VELOCITY_THRESHOLD = 1.0;
}
