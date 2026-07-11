package jsim.field;

import com.fasterxml.jackson.databind.JsonNode;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import jsim.field.SeasonFieldSpec.AprilTagSpec;
import jsim.field.SeasonFieldSpec.ColliderSpec;
import jsim.field.SeasonFieldSpec.FieldElement;
import jsim.field.SeasonFieldSpec.FieldSize;
import jsim.field.SeasonFieldSpec.GamePieceSet;
import jsim.field.SeasonFieldSpec.MaterialSpec;
import jsim.field.SeasonFieldSpec.PoseSpec;

/** Parses both canonical and legacy field JSON into {@link SeasonFieldSpec}. */
public final class SeasonFieldSpecParser {
    private SeasonFieldSpecParser() {}

    public static SeasonFieldSpec parse(JsonNode root, String fallbackId) {
        int schemaVersion = root.path("schemaVersion").asInt(1);

        JsonNode seasonNode = root.path("season");
        String id = seasonNode.path("id").asText(fallbackId);
        int year = seasonNode.path("year").asInt(root.path("year").asInt(0));
        String game = seasonNode.path("game").asText(root.path("game").asText(""));

        JsonNode fieldNode = root.path("field");
        FieldSize field = new FieldSize(
                fieldNode.path("lengthMeters").asDouble(FieldLayout.FRC_LENGTH_M),
                fieldNode.path("widthMeters").asDouble(FieldLayout.FRC_WIDTH_M));

        List<FieldElement> elements = parseElements(root);
        List<AprilTagSpec> aprilTags = parseAprilTags(root.path("aprilTags"));
        List<GamePieceSet> gamePieceSets = parseGamePieceSets(root);
        Map<String, String> renderData = parseStringMap(root.path("render"));

        return new SeasonFieldSpec(schemaVersion, id, year, game, field, elements, aprilTags, gamePieceSets, renderData);
    }

    private static List<FieldElement> parseElements(JsonNode root) {
        if (root.has("elements")) {
            return parseCanonicalElements(root.path("elements"));
        }
        // Legacy fallback: map obstacles[] => static box elements[].
        List<FieldElement> out = new ArrayList<>();
        for (JsonNode obs : root.path("obstacles")) {
            String name = obs.path("name").asText("Obstacle");
            PoseSpec pose = PoseSpec.of(
                    obs.path("x").asDouble(),
                    obs.path("y").asDouble(),
                    obs.path("z").asDouble());
            ColliderSpec collider = ColliderSpec.box(
                    obs.path("halfX").asDouble(),
                    obs.path("halfY").asDouble(),
                    obs.path("halfZ").asDouble());
            out.add(new FieldElement(
                    name,
                    "obstacle",
                    pose,
                    collider,
                    MaterialSpec.wall(),
                    false,
                    List.of(),
                    Map.of()));
        }
        return out;
    }

    private static List<FieldElement> parseCanonicalElements(JsonNode elementsNode) {
        List<FieldElement> out = new ArrayList<>();
        for (JsonNode n : elementsNode) {
            String name = n.path("name").asText("Element");
            String kind = n.path("kind").asText("obstacle");
            JsonNode poseNode = n.path("pose");
            PoseSpec pose = new PoseSpec(
                    poseNode.path("x").asDouble(),
                    poseNode.path("y").asDouble(),
                    poseNode.path("z").asDouble(),
                    poseNode.path("rollRadians").asDouble(),
                    poseNode.path("pitchRadians").asDouble(),
                    poseNode.path("yawRadians").asDouble());
            JsonNode c = n.path("collider");
            ColliderSpec collider = new ColliderSpec(
                    c.path("shape").asText("box"),
                    c.path("halfX").asDouble(),
                    c.path("halfY").asDouble(),
                    c.path("halfZ").asDouble(),
                    c.path("radius").asDouble(),
                    c.path("nx").asDouble(),
                    c.path("ny").asDouble(),
                    c.path("nz").asDouble(),
                    c.path("offset").asDouble());
            JsonNode materialNode = n.path("material");
            MaterialSpec material = new MaterialSpec(
                    materialNode.path("name").asText("WALL"),
                    materialNode.path("friction").asDouble(0.7),
                    materialNode.path("restitution").asDouble(0.2));

            List<String> zoneTags = new ArrayList<>();
            for (JsonNode tag : n.path("zoneTags")) {
                zoneTags.add(tag.asText());
            }
            out.add(new FieldElement(
                    name,
                    kind,
                    pose,
                    collider,
                    material,
                    n.path("mirrorAcrossCenterline").asBoolean(false),
                    zoneTags,
                    parseStringMap(n.path("metadata"))));
        }
        return out;
    }

    private static List<AprilTagSpec> parseAprilTags(JsonNode tagsNode) {
        List<AprilTagSpec> out = new ArrayList<>();
        for (JsonNode t : tagsNode) {
            out.add(new AprilTagSpec(
                    t.path("id").asInt(),
                    t.path("x").asDouble(),
                    t.path("y").asDouble(),
                    t.path("z").asDouble(),
                    t.path("rollRadians").asDouble(),
                    t.path("pitchRadians").asDouble(),
                    t.path("yawRadians").asDouble()));
        }
        return out;
    }

    private static List<GamePieceSet> parseGamePieceSets(JsonNode root) {
        JsonNode sets = root.has("gamePieceSets") ? root.path("gamePieceSets") : root.path("gamePieces");
        List<GamePieceSet> out = new ArrayList<>();
        for (JsonNode type : sets) {
            String variant = type.path("variant").asText();
            double radius = type.path("radius").asDouble();
            double mass = type.path("mass").asDouble();
            double friction = type.path("friction").asDouble(0.5);
            double restitution = type.path("restitution").asDouble(0.3);
            List<PoseSpec> poses = new ArrayList<>();
            for (JsonNode pose : type.path("poses")) {
                poses.add(PoseSpec.of(
                        pose.path("x").asDouble(),
                        pose.path("y").asDouble(),
                        pose.path("z").asDouble()));
            }
            out.add(new GamePieceSet(variant, radius, mass, friction, restitution, poses));
        }
        return out;
    }

    private static Map<String, String> parseStringMap(JsonNode n) {
        Map<String, String> out = new HashMap<>();
        if (n == null || n.isMissingNode() || !n.isObject()) {
            return out;
        }
        n.fields().forEachRemaining(e -> out.put(e.getKey(), e.getValue().asText()));
        return out;
    }
}
