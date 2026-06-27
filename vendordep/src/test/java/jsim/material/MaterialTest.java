package jsim.material;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class MaterialTest {

    private static final double EPS = 1e-12;

    // Material construction

    @Test
    void construction_validInputs() {
        Material m = new Material(0.5, 0.3);
        assertEquals(0.5, m.friction, EPS);
        assertEquals(0.3, m.restitution, EPS);
    }

    @Test
    void construction_zeroFrictionAllowed() {
        assertDoesNotThrow(() -> new Material(0.0, 0.0));
    }

    @Test
    void construction_negativeFrictionThrows() {
        assertThrows(IllegalArgumentException.class, () -> new Material(-0.1, 0.5));
    }

    @Test
    void construction_restitutionAboveOneThrows() {
        assertThrows(IllegalArgumentException.class, () -> new Material(0.5, 1.1));
    }

    @Test
    void construction_negativeRestitutionThrows() {
        assertThrows(IllegalArgumentException.class, () -> new Material(0.5, -0.01));
    }

    @Test
    void construction_restitutionExactlyOneAllowed() {
        assertDoesNotThrow(() -> new Material(0.5, 1.0));
    }

    // MaterialCombiner — friction
    @Test
    void combineFriction_geometricMean() {
        Material a = new Material(0.25, 0.5);
        Material b = new Material(0.16, 0.5);
        // sqrt(0.25 * 0.16) = sqrt(0.04) = 0.2
        assertEquals(0.2, MaterialCombiner.combineFriction(a, b), EPS);
    }

    @Test
    void combineFriction_zeroFrictionSurfaceGivesZero() {
        Material a = new Material(0.0, 0.5);
        Material b = new Material(0.9, 0.5);
        assertEquals(0.0, MaterialCombiner.combineFriction(a, b), EPS);
    }

    @Test
    void combineFriction_identicalSurfaces() {
        // sqrt(x * x) = x
        Material a = new Material(0.7, 0.5);
        assertEquals(0.7, MaterialCombiner.combineFriction(a, a), EPS);
    }

    @Test
    void combineFriction_isCommutative() {
        Material a = new Material(0.3, 0.5);
        Material b = new Material(0.6, 0.5);
        assertEquals(
            MaterialCombiner.combineFriction(a, b),
            MaterialCombiner.combineFriction(b, a), EPS);
    }

    // MaterialCombiner — restitution
    @Test
    void combineRestitution_takesMinimum() {
        Material a = new Material(0.5, 0.8);
        Material b = new Material(0.5, 0.3);
        assertEquals(0.3, MaterialCombiner.combineRestitution(a, b), EPS);
    }

    @Test
    void combineRestitution_zeroRestitutionSurfaceGivesZero() {
        Material a = new Material(0.5, 0.0);
        Material b = new Material(0.5, 0.9);
        assertEquals(0.0, MaterialCombiner.combineRestitution(a, b), EPS);
    }

    @Test
    void combineRestitution_isCommutative() {
        Material a = new Material(0.5, 0.4);
        Material b = new Material(0.5, 0.7);
        assertEquals(
            MaterialCombiner.combineRestitution(a, b),
            MaterialCombiner.combineRestitution(b, a), EPS);
    }

    // Combined material
    @Test
    void combine_producesCorrectBothProperties() {
        Material a = new Material(0.49, 0.9);
        Material b = new Material(0.49, 0.5);
        Material c = MaterialCombiner.combine(a, b);
        // friction: sqrt(0.49 * 0.49) = 0.49
        assertEquals(0.49, c.friction, EPS);
        // restitution: min(0.9, 0.5) = 0.5
        assertEquals(0.5, c.restitution, EPS);
    }

    // Preset materials sanity checks

    @Test
    void presets_haveValidRanges() {
        for (Material m : new Material[]{
                Material.DEFAULT, Material.ICE, Material.RUBBER,
                Material.STEEL, Material.CARPET, Material.WOOD, Material.WALL}) {
            assertTrue(m.friction >= 0, "friction must be >= 0");
            assertTrue(m.restitution >= 0 && m.restitution <= 1, "restitution must be in [0,1]");
        }
    }

    @Test
    void rubber_hasHigherFrictionThanIce() {
        assertTrue(Material.RUBBER.friction > Material.ICE.friction);
    }
}
