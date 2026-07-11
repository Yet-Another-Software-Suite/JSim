package jsim.simulation;

import java.util.List;

/**
 * Immutable season-pack configuration for {@link SimulatedArena}.
 *
 * @param id canonical season identifier (for example {@code reefscape2025})
 * @param version schema version for migration safety
 * @param fieldResource bundled field resource name under {@code /jsim/field}
 * @param gamePieceVariants default game-piece variants for this season
 */
public record SeasonConfig(
        String id,
        int version,
        String fieldResource,
        List<String> gamePieceVariants) {

    public SeasonConfig {
        gamePieceVariants = List.copyOf(gamePieceVariants);
    }

    /** Stable serialization key that includes explicit version. */
    public String key() {
        return id + "@v" + version;
    }
}
