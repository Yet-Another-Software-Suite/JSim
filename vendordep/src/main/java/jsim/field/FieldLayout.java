package jsim.field;

import jsim.api.SimBodyBuilder;
import jsim.api.SimWorld;
import jsim.material.Material;

/**
 * Utility for populating a {@link SimWorld} with FRC field geometry: a carpet floor
 * (infinite plane) and four perimeter wall segments (box colliders).
 *
 * <p>All bodies added by this class are static — they deflect game pieces and robots
 * but cannot themselves be moved.
 *
 * <p>Coordinate frame: X points from the blue alliance wall toward the red alliance wall;
 * Y points across the field; Z points up. This matches AdvantageScope's field rendering.
 */
public final class FieldLayout {
    /** FRC 2025/2026 field length (blue to red) in metres (54 ft). */
    public static final double FRC_LENGTH_M = 16.4592;
    /** FRC 2025/2026 field width in metres (27 ft). */
    public static final double FRC_WIDTH_M  =  8.2296;

    private static final double WALL_THICK  = 0.05;
    private static final double WALL_HEIGHT = 0.5;

    private FieldLayout() {}

    /**
     * Add a standard FRC field (carpet floor + four perimeter walls) to {@code world}.
     *
     * @param world the world to populate
     */
    public static void addFrcField(SimWorld world) {
        addField(world, FRC_LENGTH_M, FRC_WIDTH_M);
    }

    /**
     * Add a configurable rectangular field (carpet floor + four perimeter walls) to {@code world}.
     *
     * @param world    the world to populate
     * @param lengthM  field length along X (metres)
     * @param widthM   field width along Y (metres)
     */
    public static void addField(SimWorld world, double lengthM, double widthM) {
        addFloor(world);
        addWalls(world, lengthM, widthM, WALL_THICK, WALL_HEIGHT);
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
     * Add four perimeter wall segments to {@code world}.
     *
     * <p>The blue and red end walls are extended in Y to cover the field corners,
     * so no gap exists at any corner of the boundary.
     *
     * @param world     the world to populate
     * @param lengthM   field length along X (metres)
     * @param widthM    field width along Y (metres)
     * @param thickM    wall thickness (metres)
     * @param heightM   wall height above the floor (metres)
     */
    public static void addWalls(SimWorld world,
                                 double lengthM, double widthM,
                                 double thickM,  double heightM) {
        double t2  = thickM  / 2.0;
        double h2  = heightM / 2.0;
        double l2  = lengthM / 2.0;
        double w2  = widthM  / 2.0;

        // Blue alliance wall — inner face at x = 0, box extends into x < 0.
        // halfY is w2+thickM so it overlaps the adjacent Near/Far wall thickness,
        // closing the corner gap.
        world.addBody(new SimBodyBuilder("Field/WallBlue")
            .position(-t2, w2, h2)
            .boxCollider(t2, w2 + thickM, h2)
            .material(Material.WALL)
            .isStatic());

        // Red alliance wall — inner face at x = lengthM.
        world.addBody(new SimBodyBuilder("Field/WallRed")
            .position(lengthM + t2, w2, h2)
            .boxCollider(t2, w2 + thickM, h2)
            .material(Material.WALL)
            .isStatic());

        // Near wall — inner face at y = 0, box extends into y < 0.
        world.addBody(new SimBodyBuilder("Field/WallNear")
            .position(l2, -t2, h2)
            .boxCollider(l2, t2, h2)
            .material(Material.WALL)
            .isStatic());

        // Far wall — inner face at y = widthM.
        world.addBody(new SimBodyBuilder("Field/WallFar")
            .position(l2, widthM + t2, h2)
            .boxCollider(l2, t2, h2)
            .material(Material.WALL)
            .isStatic());
    }
}
