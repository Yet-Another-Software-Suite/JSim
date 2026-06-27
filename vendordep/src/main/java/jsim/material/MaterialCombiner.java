package jsim.material;

/**
 * Combines two surface materials into a single contact response.
 *
 * <p>Friction is combined geometrically (sqrt product), restitution by the minimum, matching
 * common game-engine conventions that produce stable, physically plausible results.
 */
public final class MaterialCombiner {
    private MaterialCombiner() {}

    /**
     * Combined coefficient of kinetic friction for a pair of surfaces.
     *
     * <p>Uses the geometric mean so that a zero-friction surface always produces zero friction,
     * while two high-friction surfaces produce high combined friction.
     *
     * @param a first surface material
     * @param b second surface material
     * @return combined friction coefficient
     */
    public static double combineFriction(Material a, Material b) {
        return Math.sqrt(a.friction * b.friction);
    }

    /**
     * Combined coefficient of restitution for a pair of surfaces.
     *
     * <p>Uses the minimum so that a fully inelastic body always absorbs all energy at the contact,
     * regardless of the other body's restitution.
     *
     * @param a first surface material
     * @param b second surface material
     * @return combined restitution coefficient
     */
    public static double combineRestitution(Material a, Material b) {
        return Math.min(a.restitution, b.restitution);
    }

    /**
     * Convenience: produce a new {@link Material} from two surfaces.
     *
     * @param a first surface material
     * @param b second surface material
     * @return combined material
     */
    public static Material combine(Material a, Material b) {
        return new Material(combineFriction(a, b), combineRestitution(a, b));
    }
}
