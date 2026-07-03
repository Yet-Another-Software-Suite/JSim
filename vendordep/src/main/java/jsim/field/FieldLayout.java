package jsim.field;

import jsim.api.SimBodyBuilder;
import jsim.api.SimWorld;
import jsim.material.Material;

/**
 * Utility for populating a {@link SimWorld} with FRC field geometry: a carpet floor
 * and four perimeter wall planes.
 *
 * <p>All bodies added by this class are static — they deflect game pieces and robots
 * but cannot themselves be moved.
 *
 * <p><b>Coordinate frame:</b> X points from the blue alliance wall toward the red alliance
 * wall; Y points across the field; Z points up. This matches AdvantageScope's field rendering.
 *
 * <p><b>Wall implementation:</b> Each wall is an infinite half-space {@link jsim.collision.PlaneCollider}.
 * Infinite planes are always in the broadphase and use signed-distance narrowphase, so they are
 * completely immune to tunneling even when game pieces are launched at very high velocities.
 */
public final class FieldLayout {
    /** FRC 2025/2026 field length (blue to red) in metres (54 ft). */
    public static final double FRC_LENGTH_M = 16.4592;
    /** FRC 2025/2026 field width in metres (27 ft). */
    public static final double FRC_WIDTH_M  =  8.2296;

    private FieldLayout() {}

    /**
     * Add a standard FRC field (carpet floor + four perimeter wall planes) to {@code world}.
     *
     * @param world the world to populate
     */
    public static void addFrcField(SimWorld world) {
        addField(world, FRC_LENGTH_M, FRC_WIDTH_M);
    }

    /**
     * Add a configurable rectangular field (carpet floor + four perimeter wall planes) to
     * {@code world}.
     *
     * @param world    the world to populate
     * @param lengthM  field length along X (metres)
     * @param widthM   field width along Y (metres)
     */
    public static void addField(SimWorld world, double lengthM, double widthM) {
        addFloor(world);
        addWalls(world, lengthM, widthM);
    }

    /**
     * Add just the carpet floor (infinite plane at Z = 0) to {@code world}.
     *
     * @param world the world to populate
     */
    public static void addFloor(SimWorld world) {
        world.addBody(new SimBodyBuilder("Field/Floor")
            .planeCollider(0, 0, 1, 0)
            .material(Material.CARPET));
    }

    /**
     * Add four perimeter wall planes to {@code world}.
     *
     * <p>Each wall is a {@link jsim.collision.PlaneCollider} (infinite half-space) anchored
     * at the corresponding field boundary and oriented so its normal points into the field.
     * Plane colliders have unbounded AABBs and use signed-distance collision tests, making
     * them immune to tunneling at any projectile speed.
     *
     * @param world    the world to populate
     * @param lengthM  field length along X (metres)
     * @param widthM   field width along Y (metres)
     */
    public static void addWalls(SimWorld world, double lengthM, double widthM) {
        // Blue alliance wall — inner face at x = 0, normal points +X into field.
        // Blocks the half-space x < 0.
        world.addBody(new SimBodyBuilder("Field/WallBlue")
            .planeCollider(1, 0, 0, 0)
            .material(Material.WALL));

        // Red alliance wall — inner face at x = lengthM, normal points -X into field.
        // Body is placed at (lengthM, 0, 0); offset = 0, so the plane passes through
        // that point.  Blocks the half-space x > lengthM.
        world.addBody(new SimBodyBuilder("Field/WallRed")
            .position(lengthM, 0, 0)
            .planeCollider(-1, 0, 0, 0)
            .material(Material.WALL));

        // Near wall — inner face at y = 0, normal points +Y into field.
        // Blocks the half-space y < 0.
        world.addBody(new SimBodyBuilder("Field/WallNear")
            .planeCollider(0, 1, 0, 0)
            .material(Material.WALL));

        // Far wall — inner face at y = widthM, normal points -Y into field.
        // Blocks the half-space y > widthM.
        world.addBody(new SimBodyBuilder("Field/WallFar")
            .position(0, widthM, 0)
            .planeCollider(0, -1, 0, 0)
            .material(Material.WALL));
    }
}
