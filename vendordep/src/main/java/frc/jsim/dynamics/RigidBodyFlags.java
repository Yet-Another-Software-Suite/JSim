package frc.jsim.dynamics;

/** Bit-flag constants that control how a body participates in the simulation. */
public final class RigidBodyFlags {
    private RigidBodyFlags() {}

    /** Body responds to forces and collisions (default for dynamic objects). */
    public static final int DYNAMIC = 0;

    /**
     * Body has infinite mass and does not move.
     * Forces and impulses are still received by colliding dynamic bodies,
     * but this body's state never changes.
     */
    public static final int STATIC = 1 << 0;

    /**
     * Body is not affected by gravity.
     * Useful for neutrally-buoyant game pieces or wheel-driven robots.
     */
    public static final int NO_GRAVITY = 1 << 1;

    /**
     * Body does not participate in collision detection or response.
     * It still receives forces and integrates, so it can be used as a
     * ghost / trigger volume when combined with a sensor implementation.
     */
    public static final int NO_COLLISION = 1 << 2;

    /** Body does not rotate (infinite rotational inertia). */
    public static final int FIXED_ROTATION = 1 << 3;

    /**
     * Test whether a specific flag bit is set in a bitmask.
     *
     * @param flags bitmask to test
     * @param flag  single flag constant to check
     * @return {@code true} if the flag bit is set in the bitmask
     */
    public static boolean isSet(int flags, int flag) {
        return (flags & flag) != 0;
    }
}
