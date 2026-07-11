package jsim.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertNull;
import org.junit.jupiter.api.Test;

class SeasonRegistryTest {
    @Test
    void resolvesCanonicalAndAliasKeys() {
        SeasonConfig canonical = SeasonRegistry.resolve("reefscape2025@v1");
        SeasonConfig alias = SeasonRegistry.resolve("2025-reefscape");
        assertEquals(canonical.key(), alias.key());
        assertEquals("reefscape2025", canonical.id());
        assertEquals(1, canonical.version());
        assertNull(canonical.fieldConstantsClass());
    }

    @Test
    void resolvesNewCanonicalSeasonSpecPack() {
        SeasonConfig season = SeasonRegistry.resolve("2026-hubrush");
        assertEquals("hubrush2026", season.id());
        assertEquals("2026-hubrush", season.fieldResource());
    }

    @Test
    void unknownSeasonThrows() {
        assertThrows(IllegalArgumentException.class, () -> SeasonRegistry.resolve("unknown-season"));
    }
}
