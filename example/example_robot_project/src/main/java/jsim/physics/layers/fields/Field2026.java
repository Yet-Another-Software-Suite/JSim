package jsim.physics.layers.fields;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.dyn4j.dynamics.Body;
import org.dyn4j.world.World;

import jsim.physics.layers.FieldLayout;
// use element for 2d bump sim?
import jsim.physics.layers.FieldLayout.Element;

/**
 * Static field geometry for the 2026 FIRST Robotics Competition game, REBUILT.
 *
 * <p>Dimensions below are the nominal values from Section 5 (ARENA) of the 2026 Game Manual, not
 * official CAD, so treat structure placement as a close approximation rather than exact. Only
 * structures large enough to matter for drive collisions are modeled (HUB, BUMP, TOWER base, and
 * the field perimeter) -- thin items like tape lines, AprilTag panels, and the ~1in DEPOT barrier
 * are omitted. TRENCHES are also omitted since robots drive underneath them (22.25in of
 * clearance) rather than colliding with them. BUMPS are omitted since the robots drive over them.
 *
 * <p>Field coordinate convention (matches WPILib): origin at the blue ALLIANCE WALL corner at the
 * Y=0 guardrail, +X towards the red ALLIANCE WALL, +Y across the width of the FIELD.
 */
public class Field2026 implements FieldLayout {

  private static final double FIELD_LENGTH = Inches.of(651.2).in(Meters);
  private static final double FIELD_WIDTH = Inches.of(317.7).in(Meters);
  private static final double WALL_THICKNESS = Inches.of(2.0).in(Meters);

  private static final double HUB_SIZE = Inches.of(47.0).in(Meters);
  private static final double HUB_DISTANCE_FROM_WALL = Inches.of(158.6).in(Meters);

  private static final double BUMP_WIDTH = Inches.of(73.0).in(Meters); // along field width (Y)
  private static final double BUMP_DEPTH = Inches.of(44.4).in(Meters); // along field length (X)

  private static final double TOWER_WIDTH = Inches.of(39.0).in(Meters); // TOWER BASE, along Y
  private static final double TOWER_DEPTH = Inches.of(45.18).in(Meters); // TOWER BASE, along X

  private final FieldLayout delegate;

  public Field2026() {
    List<Element> elements = new ArrayList<>();
    elements.addAll(boundary());
    elements.addAll(allianceStructures(0.0, 1.0));
    elements.addAll(allianceStructures(FIELD_LENGTH, -1.0));
    this.delegate = FieldLayout.of(elements.toArray(new Element[0]));
  }

  @Override
  public void populateWorld(World<Body> world) {
    delegate.populateWorld(world);
  }

  /** Field perimeter: the two long guardrails and the two alliance walls. */
  private static List<Element> boundary() {
    return Arrays.asList(
        FieldLayout.rectangle(FIELD_LENGTH / 2.0, 0, FIELD_LENGTH, WALL_THICKNESS),
        FieldLayout.rectangle(FIELD_LENGTH / 2.0, FIELD_WIDTH, FIELD_LENGTH, WALL_THICKNESS),
        FieldLayout.rectangle(0, FIELD_WIDTH / 2.0, WALL_THICKNESS, FIELD_WIDTH),
        FieldLayout.rectangle(FIELD_LENGTH, FIELD_WIDTH / 2.0, WALL_THICKNESS, FIELD_WIDTH));
  }

  /**
   * Builds one alliance's HUB / BUMP / TOWER row.
   *
   * @param wallX X coordinate of this alliance's wall.
   * @param towardsCenter {@code +1} if the field extends in {@code +X} from this wall, {@code -1}
   *     if it extends in {@code -X} (i.e. this is the far alliance wall).
   */
  private static List<Element> allianceStructures(double wallX, double towardsCenter) {
    double rowX = wallX + towardsCenter * HUB_DISTANCE_FROM_WALL;
    double centerY = FIELD_WIDTH / 2.0;
    double bumpOffset = (HUB_SIZE + BUMP_WIDTH) / 2.0;
    double towerX = wallX + towardsCenter * TOWER_DEPTH / 2.0;

    return Arrays.asList(
        FieldLayout.rectangle(rowX, centerY, HUB_SIZE, HUB_SIZE),
        // FieldLayout.rectangle(rowX, centerY + bumpOffset, BUMP_DEPTH, BUMP_WIDTH),
        // FieldLayout.rectangle(rowX, centerY - bumpOffset, BUMP_DEPTH, BUMP_WIDTH),
        FieldLayout.rectangle(towerX, centerY, TOWER_DEPTH, TOWER_WIDTH));
  }
}
