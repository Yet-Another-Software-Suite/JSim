package jsim.field;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Path;
import jsim.api.SimBodyBuilder;
import jsim.core.SimConstants;
import jsim.material.Material;

/**
 * Loads a JSim field definition from a JSON resource or file, producing a fully-configured
 * {@link FieldSimulator} that includes:
 * <ul>
 *   <li>the correct perimeter walls for the field dimensions declared in the JSON,
 *   <li>static obstacle hitboxes (reef structures, scoring targets, etc.), and
 *   <li>pre-spawned game pieces at their starting positions.
 * </ul>
 *
 * <p>Bundled season definitions ship with JSim as classpath resources under
 * {@code jsim/field/<name>.json}. Load them by name:
 * <pre>{@code
 * FieldSimulator sim = FieldLoader.fromResource("2025-reefscape");
 * }</pre>
 *
 * <p>Custom field files can be loaded from disk:
 * <pre>{@code
 * FieldSimulator sim = FieldLoader.fromFile(Path.of("my-field.json"));
 * }</pre>
 *
 * <h2>JSON schema</h2>
 * <pre>{@code
 * {
 *   "year": 2025,
 *   "game": "Reefscape",
 *   "field": {
 *     "lengthMeters": 17.548,
 *     "widthMeters": 8.052
 *   },
 *   "obstacles": [
 *     {
 *       "name": "ReefBlue",
 *       "x": 4.49,  "y": 4.03,  "z": 1.00,
 *       "halfX": 0.85, "halfY": 0.85, "halfZ": 1.00
 *     }
 *   ],
 *   "gamePieces": [
 *     {
 *       "variant": "Coral",
 *       "radius": 0.057,
 *       "mass": 0.227,
 *       "friction": 0.5,
 *       "restitution": 0.3,
 *       "poses": [
 *         { "x": 1.15, "y": 0.65, "z": 0.057 }
 *       ]
 *     }
 *   ]
 * }
 * }</pre>
 *
 * <p>Each {@code obstacle} becomes a static {@link jsim.collision.BoxCollider} body in the
 * physics world — robots and game pieces bounce off it but cannot push it.  {@code gamePieces}
 * become dynamic sphere bodies tracked by {@link FieldSimulator#getGamePiecePoses(String)} for
 * AdvantageScope output.
 */
public final class FieldLoader {
    private static final ObjectMapper MAPPER = new ObjectMapper();

    private FieldLoader() {}

    /**
     * Load a bundled field definition by short name, e.g. {@code "2025-reefscape"}.
     * The classpath resource must exist at {@code /jsim/field/<name>.json}.
     *
     * @param name resource name, without path prefix or {@code .json} extension
     * @return a fully configured {@link FieldSimulator}
     * @throws IllegalArgumentException if no bundled resource matches {@code name}
     * @throws RuntimeException if the resource cannot be read or parsed
     */
    public static FieldSimulator fromResource(String name) {
        String resourcePath = "/jsim/field/" + name + ".json";
        try (InputStream is = FieldLoader.class.getResourceAsStream(resourcePath)) {
            if (is == null) {
                throw new IllegalArgumentException(
                        "No bundled field resource found at classpath:" + resourcePath);
            }
            return parse(MAPPER.readTree(is));
        } catch (IOException e) {
            throw new RuntimeException("Failed to load field resource: " + resourcePath, e);
        }
    }

    /**
     * Load a field definition from a JSON file on disk.
     *
     * @param jsonPath path to the {@code .json} field definition file
     * @return a fully configured {@link FieldSimulator}
     * @throws RuntimeException if the file cannot be read or parsed
     */
    public static FieldSimulator fromFile(Path jsonPath) {
        try {
            return parse(MAPPER.readTree(jsonPath.toFile()));
        } catch (IOException e) {
            throw new RuntimeException("Failed to load field file: " + jsonPath, e);
        }
    }

    private static FieldSimulator parse(JsonNode root) {
        // Field dimensions — fall back to standard FRC constants if absent.
        JsonNode fieldNode = root.path("field");
        double lengthM = fieldNode.path("lengthMeters").asDouble(FieldLayout.FRC_LENGTH_M);
        double widthM  = fieldNode.path("widthMeters").asDouble(FieldLayout.FRC_WIDTH_M);

        FieldSimulator sim = new FieldSimulator(SimConstants.DEFAULT_DT, lengthM, widthM);

        // Static obstacle hitboxes (reef structures, coral stations, etc.).
        // Each becomes a static box body that absorbs collisions without moving.
        for (JsonNode obs : root.path("obstacles")) {
            String name  = obs.path("name").asText("Obstacle");
            double x     = obs.path("x").asDouble();
            double y     = obs.path("y").asDouble();
            double z     = obs.path("z").asDouble();
            double halfX = obs.path("halfX").asDouble();
            double halfY = obs.path("halfY").asDouble();
            double halfZ = obs.path("halfZ").asDouble();
            sim.getWorld().addBody(new SimBodyBuilder(name)
                    .position(x, y, z)
                    .boxCollider(halfX, halfY, halfZ)
                    .isStatic()
                    .material(Material.WALL));
        }

        // Dynamic game piece bodies tracked for AdvantageScope output.
        for (JsonNode type : root.path("gamePieces")) {
            String   variant     = type.path("variant").asText();
            double   radius      = type.path("radius").asDouble();
            double   mass        = type.path("mass").asDouble();
            double   friction    = type.path("friction").asDouble(0.5);
            double   restitution = type.path("restitution").asDouble(0.3);
            Material mat         = new Material(friction, restitution);
            int      i           = 0;
            for (JsonNode pose : type.path("poses")) {
                double px = pose.path("x").asDouble();
                double py = pose.path("y").asDouble();
                double pz = pose.path("z").asDouble();
                sim.spawnGamePiece(variant, new SimBodyBuilder(variant + i)
                        .position(px, py, pz)
                        .mass(mass)
                        .sphereCollider(radius)
                        .material(mat));
                i++;
            }
        }

        return sim;
    }
}
