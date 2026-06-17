package frc.jsim.material;

/**
 * Physical surface properties for collision response.
 *
 * <p>Friction and restitution are dimensionless coefficients in [0, 1] unless otherwise noted.
 * Coefficient of friction > 1 is physically valid (rubber on rubber) and is allowed.
 */
public final class Material {
    /** Coefficient of kinetic friction. */
    public final double friction;

    /** Coefficient of restitution (0 = perfectly inelastic, 1 = perfectly elastic). */
    public final double restitution;

    /** General-purpose material with moderate friction and low restitution. */
    public static final Material DEFAULT   = new Material(0.5,  0.3);
    /** Low-friction icy surface. */
    public static final Material ICE       = new Material(0.03, 0.1);
    /** High-friction, high-restitution rubber. */
    public static final Material RUBBER    = new Material(0.9,  0.7);
    /** Low-friction steel surface. */
    public static final Material STEEL     = new Material(0.3,  0.2);
    /** High-friction carpet (typical FRC field surface). */
    public static final Material CARPET    = new Material(0.8,  0.1);
    /** Moderate-friction wood surface. */
    public static final Material WOOD      = new Material(0.5,  0.4);
    /** Infinite-mass static boundary — frictionless, inelastic. */
    public static final Material WALL      = new Material(0.2,  0.05);

    /**
     * Create a material with the given friction and restitution coefficients.
     *
     * @param friction    coefficient of kinetic friction (must be &gt;= 0)
     * @param restitution coefficient of restitution, in [0, 1]
     */
    public Material(double friction, double restitution) {
        if (friction < 0) throw new IllegalArgumentException("friction must be >= 0");
        if (restitution < 0 || restitution > 1) {
            throw new IllegalArgumentException("restitution must be in [0, 1]");
        }
        this.friction = friction;
        this.restitution = restitution;
    }
}
