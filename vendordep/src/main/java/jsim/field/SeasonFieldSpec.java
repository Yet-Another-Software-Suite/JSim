package jsim.field;

import java.util.List;
import java.util.Map;

/** Canonical schema for season field definition data. */
public record SeasonFieldSpec(
        int schemaVersion,
        String id,
        int year,
        String game,
        FieldSize field,
        List<FieldElement> elements,
        List<AprilTagSpec> aprilTags,
        List<GamePieceSet> gamePieceSets,
        Map<String, String> renderData) {

    public SeasonFieldSpec {
        elements = List.copyOf(elements);
        aprilTags = List.copyOf(aprilTags);
        gamePieceSets = List.copyOf(gamePieceSets);
        renderData = Map.copyOf(renderData);
    }

    public record FieldSize(double lengthMeters, double widthMeters) {}

    public record PoseSpec(
            double x,
            double y,
            double z,
            double rollRadians,
            double pitchRadians,
            double yawRadians) {
        public static PoseSpec of(double x, double y, double z) {
            return new PoseSpec(x, y, z, 0.0, 0.0, 0.0);
        }
    }

    public record MaterialSpec(String name, double friction, double restitution) {
        public static MaterialSpec wall() {
            return new MaterialSpec("WALL", 0.7, 0.2);
        }
    }

    public record ColliderSpec(
            String shape, // "box", "sphere", "plane"
            double halfX,
            double halfY,
            double halfZ,
            double radius,
            double nx,
            double ny,
            double nz,
            double offset) {
        public static ColliderSpec box(double halfX, double halfY, double halfZ) {
            return new ColliderSpec("box", halfX, halfY, halfZ, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        public static ColliderSpec sphere(double radius) {
            return new ColliderSpec("sphere", 0.0, 0.0, 0.0, radius, 0.0, 0.0, 0.0, 0.0);
        }

        public static ColliderSpec plane(double nx, double ny, double nz, double offset) {
            return new ColliderSpec("plane", 0.0, 0.0, 0.0, 0.0, nx, ny, nz, offset);
        }
    }

    public record FieldElement(
            String name,
            String kind,
            PoseSpec pose,
            ColliderSpec collider,
            MaterialSpec material,
            boolean mirrorAcrossCenterline,
            List<String> zoneTags,
            Map<String, String> metadata) {
        public FieldElement {
            zoneTags = List.copyOf(zoneTags);
            metadata = Map.copyOf(metadata);
        }
    }

    public record AprilTagSpec(
            int id, double x, double y, double z, double rollRadians, double pitchRadians, double yawRadians) {}

    public record GamePieceSet(
            String variant,
            double radius,
            double mass,
            double friction,
            double restitution,
            List<PoseSpec> poses) {
        public GamePieceSet {
            poses = List.copyOf(poses);
        }
    }
}
