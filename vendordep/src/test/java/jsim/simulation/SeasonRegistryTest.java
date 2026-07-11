package jsim.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import org.junit.jupiter.api.Test;

class SeasonRegistryTest {
    @Test
    void resolvesCanonicalAndAliasKeys() {
        SeasonConfig canonical = SeasonRegistry.resolve("reefscape2025@v1");
        SeasonConfig alias = SeasonRegistry.resolve("2025-reefscape");
        assertEquals(canonical.key(), alias.key());
        assertEquals("reefscape2025", canonical.id());
        assertEquals(1, canonical.version());
    }

    @Test
    void unknownSeasonThrows() {
        assertThrows(IllegalArgumentException.class, () -> SeasonRegistry.resolve("unknown-season"));
    }
}
