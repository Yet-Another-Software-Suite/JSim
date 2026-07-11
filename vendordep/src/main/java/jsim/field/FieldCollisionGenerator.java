package jsim.field;

import edu.wpi.first.math.geometry.Rotation3d;
import jsim.api.SimBodyBuilder;
import jsim.material.Material;

/** Builds static colliders in the world from canonical field elements. */
public final class FieldCollisionGenerator {
    private FieldCollisionGenerator() {}

    public static void addElementColliders(FieldSimulator sim, SeasonFieldSpec spec) {
        for (SeasonFieldSpec.FieldElement element : spec.elements()) {
            addElement(sim, spec, element, false);
            if (element.mirrorAcrossCenterline()) {
                addElement(sim, spec, mirrorAcrossCenterline(spec, element), true);
            }
        }
    }

    private static SeasonFieldSpec.FieldElement mirrorAcrossCenterline(SeasonFieldSpec spec, SeasonFieldSpec.FieldElement e) {
        SeasonFieldSpec.PoseSpec p = e.pose();
        SeasonFieldSpec.PoseSpec mirroredPose = new SeasonFieldSpec.PoseSpec(
                p.x(),
                spec.field().widthMeters() - p.y(),
                p.z(),
                p.rollRadians(),
                p.pitchRadians(),
                -p.yawRadians());
        return new SeasonFieldSpec.FieldElement(
                e.name(),
                e.kind(),
                mirroredPose,
                e.collider(),
                e.material(),
                false,
                e.zoneTags(),
                e.metadata());
    }

    private static void addElement(FieldSimulator sim, SeasonFieldSpec spec, SeasonFieldSpec.FieldElement element, boolean mirroredCopy) {
        SeasonFieldSpec.ColliderSpec c = element.collider();
        SeasonFieldSpec.PoseSpec p = element.pose();
        SimBodyBuilder builder = new SimBodyBuilder(mirroredCopy ? element.name() + "/Mirrored" : element.name())
                .position(p.x(), p.y(), p.z())
                .rotation(new Rotation3d(p.rollRadians(), p.pitchRadians(), p.yawRadians()))
                .isStatic()
                .material(toMaterial(element.material()));

        switch (c.shape()) {
            case "box" -> builder.boxCollider(c.halfX(), c.halfY(), c.halfZ());
            case "sphere" -> builder.sphereCollider(c.radius());
            case "plane" -> builder.planeCollider(c.nx(), c.ny(), c.nz(), c.offset());
            default -> throw new IllegalArgumentException("Unsupported collider shape: " + c.shape());
        }
        sim.getWorld().addBody(builder);
    }

    private static Material toMaterial(SeasonFieldSpec.MaterialSpec material) {
        if (material == null) {
            return Material.WALL;
        }
        return switch (material.name()) {
            case "CARPET" -> Material.CARPET;
            case "RUBBER" -> Material.RUBBER;
            case "STEEL", "METAL" -> Material.STEEL;
            case "WOOD" -> Material.WOOD;
            case "ICE" -> Material.ICE;
            default -> new Material(material.friction(), material.restitution());
        };
    }
}
