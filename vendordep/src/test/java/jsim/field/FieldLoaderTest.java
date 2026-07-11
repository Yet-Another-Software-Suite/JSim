package jsim.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;

class FieldLoaderTest {
    @Test
    void canonicalSeasonSpecLoadsCollidersAndGamePieces() {
        FieldSimulator sim = FieldLoader.fromResource("2026-hubrush");

        // 5 perimeter bodies + 3 generated element bodies (hub + mirrored bumps)
        assertTrue(sim.getWorld().getBodies().size() >= 8);
        assertNotNull(sim.getWorld().findBody("Hub"));
        assertNotNull(sim.getWorld().findBody("Bump"));
        assertNotNull(sim.getWorld().findBody("Bump/Mirrored"));
        assertEquals(3, sim.getGamePiecePoses("Fuel").length);
    }

    @Test
    void canonicalSpecPreservesAprilTagsAndRenderData() {
        FieldSimulator sim = FieldLoader.fromResource("2026-hubrush");
        SeasonFieldSpec spec = sim.getSeasonFieldSpec();

        assertNotNull(spec);
        assertEquals(1, spec.schemaVersion());
        assertEquals("hubrush2026", spec.id());
        assertEquals(2, spec.aprilTags().size());
        assertEquals("2026-official", spec.renderData().get("aprilTagLayout"));
    }
}
