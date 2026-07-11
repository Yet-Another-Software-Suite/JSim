package jsim.simulation;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Registry of season packs with alias-based migration support.
 *
 * <p>Callers can resolve either explicit keys ({@code reefscape2025@v1}) or legacy
 * aliases ({@code 2025-reefscape}) and always receive the latest stable config.
 */
public final class SeasonRegistry {
    private static final Map<String, SeasonConfig> PACKS = new HashMap<>();
    private static final Map<String, String> ALIASES = new HashMap<>();

    static {
        register(new SeasonConfig("reefscape2025", 1, "2025-reefscape", null, List.of("Coral", "Algae")));
        register(new SeasonConfig("crescendo2024", 1, "2025-reefscape", null, List.of("Note")));
        register(new SeasonConfig("hubrush2026", 1, "2026-hubrush", null, List.of("Fuel")));
        registerAlias("reefscape2025", "reefscape2025@v1");
        registerAlias("2025-reefscape", "reefscape2025@v1");
        registerAlias("crescendo2024", "crescendo2024@v1");
        registerAlias("hubrush2026", "hubrush2026@v1");
        registerAlias("2026-hubrush", "hubrush2026@v1");
    }

    private SeasonRegistry() {}

    public static synchronized void register(SeasonConfig config) {
        PACKS.put(config.key(), config);
    }

    public static synchronized void registerAlias(String alias, String targetKey) {
        ALIASES.put(alias, targetKey);
    }

    public static synchronized SeasonConfig resolve(String keyOrAlias) {
        SeasonConfig direct = PACKS.get(keyOrAlias);
        if (direct != null) return direct;
        String target = ALIASES.get(keyOrAlias);
        if (target == null) {
            throw new IllegalArgumentException("Unknown season pack: " + keyOrAlias);
        }
        SeasonConfig resolved = PACKS.get(target);
        if (resolved == null) {
            throw new IllegalStateException("Alias points to missing season pack: " + keyOrAlias + " -> " + target);
        }
        return resolved;
    }

    public static synchronized Collection<SeasonConfig> allPacks() {
        return Collections.unmodifiableCollection(PACKS.values());
    }
}
