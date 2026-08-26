package jsim.physics.layers.fields;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.dyn4j.dynamics.Body;
import org.dyn4j.world.World;

import jsim.physics.layers.FieldLayout;

/**
 * Static field geometry for the 2026 FIRST Robotics Competition game, REBUILT.
 *
 * <p>The field boundary and the HUB/TOWER structures are derived directly from an
 * {@link AprilTagFieldLayout} rather than hand-measured constants, since the welded and AndyMark
 * variants of the field actually differ by a few centimeters in both overall size and structure
 * placement (see {@link AprilTagFields#k2026RebuiltWelded} vs {@link AprilTagFields#k2026RebuiltAndymark}).
 * Building from whichever layout a team is actually competing on keeps this in sync automatically
 * instead of relying on a single nominal guess.
 *
 * <p>The HUB's collision shape is the actual octagon formed by its 8 AprilTags (2 per Manual
 * Section 5.11, and the tags sit centered on each face, so their positions directly trace the
 * structure's real outer footprint). The TOWER's 2 wall-mounted AprilTags only pin down a mount
 * position, not the structure's full footprint, so its UPRIGHTS use the nominal dimensions from
 * Section 5.8 of the Game Manual, centered on that derived position.
 *
 * <p>Per Section 5.8, a TOWER is <b>not</b> a solid obstacle: robots drive underneath/between its
 * RUNGS and TOWER BASE plate (which is only ~0.5-0.8cm tall), and can only actually collide with
 * its two UPRIGHTS -- the vertical posts a climbing robot grabs onto. Those are what's modeled here.
 *
 * <p>Per Sections 5.5/5.6, each TRENCH's own 65.65in width isn't fully driveable: only a 50.34in
 * corridor underneath is clear, the remaining 15.31in is the TRENCH's gate/pivot-arm attachment
 * hardware, positioned against the BUMP it connects to. That's modeled as a {@code TRENCH_GATE}
 * element per TRENCH (4 total); the rest of each TRENCH is still omitted since robots drive under
 * it otherwise.
 *
 * <p>Only structures large enough to matter for drive collisions are modeled (HUB, TOWER UPRIGHTS,
 * TRENCH GATEs, and the field perimeter) -- thin items like tape lines, AprilTag panels, and the
 * ~1in DEPOT barrier are omitted. BUMPS are omitted since robots drive over them (their nominal
 * footprint is still used to anchor each TRENCH GATE's position, since BUMPS aren't otherwise
 * modeled here).
 *
 * <p>Field coordinate convention (matches WPILib): origin at the blue ALLIANCE WALL corner at the
 * Y=0 guardrail, +X towards the red ALLIANCE WALL, +Y across the width of the FIELD.
 */
public class Field2026 implements FieldLayout {

  /** Thickness of the modeled boundary walls (guardrails and alliance walls), in meters. */
  private static final double WALL_THICKNESS = Inches.of(2.0).in(Meters);

  /** Nominal UPRIGHT cross-section depth (X axis), per Section 5.8. */
  private static final double UPRIGHT_DEPTH = Inches.of(3.5).in(Meters);

  /** Nominal UPRIGHT cross-section thickness (Y axis), per Section 5.8. */
  private static final double UPRIGHT_THICKNESS = Inches.of(1.5).in(Meters);

  /** Center-to-center spacing between a TOWER's two UPRIGHTS (Y axis), per Section 5.8. */
  private static final double UPRIGHT_SPACING = Inches.of(32.25).in(Meters);

  /** Nominal TOWER depth (X axis) used to position the UPRIGHTS relative to their wall-mount tags. */
  private static final double TOWER_DEPTH = Inches.of(45.18).in(Meters);

  /**
   * Non-passable width (X axis, parallel to the guardrail) of each TRENCH GATE -- the difference
   * between a TRENCH's full 65.65in width and its 50.34in clear driving corridor (Section 5.6).
   */
  private static final double TRENCH_GATE_WIDTH = Inches.of(15.31).in(Meters);

  /** Depth (Y axis, perpendicular to the guardrail) of each TRENCH GATE, per Section 5.6. */
  private static final double TRENCH_GATE_DEPTH = Inches.of(44.4).in(Meters);

  /**
   * Nominal BUMP width (Y axis) used only to anchor each TRENCH GATE's position against the BUMP
   * it's next to -- BUMPS themselves aren't modeled as collision elements (robots drive over them).
   */
  private static final double BUMP_WIDTH = Inches.of(73.0).in(Meters);

  // AprilTag ID groups per Game Manual Section 5.11. HUB tags are 2-per-face on all 4 faces (8
  // total), so their positions trace the structure's actual octagonal footprint. TOWER WALL tags
  // are just 2 per alliance, mounted on the wall itself, so they only give a mount position.
  private static final int[] BLUE_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};
  private static final int[] RED_HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};
  private static final int[] BLUE_TOWER_WALL_TAG_IDS = {31, 32};
  private static final int[] RED_TOWER_WALL_TAG_IDS = {15, 16};

  private final double fieldLength;
  private final double fieldWidth;
  private final Element blueHub;
  private final Element redHub;
  private final Element[] blueTowerUprights;
  private final Element[] redTowerUprights;
  private final Element blueSouthTrenchGate;
  private final Element blueNorthTrenchGate;
  private final Element redSouthTrenchGate;
  private final Element redNorthTrenchGate;
  private final Element westAllianceWall;
  private final Element eastAllianceWall;
  private final Element southGuardrail;
  private final Element northGuardrail;
  private final List<Element> elements;

  /** Builds the 2026 REBUILT field from WPILib's official welded-field AprilTag layout. */
  public Field2026() {
    this(AprilTagFields.k2026RebuiltWelded);
  }

  /**
   * Builds the 2026 REBUILT field from the given official AprilTag field variant.
   *
   * @param variant {@link AprilTagFields#k2026RebuiltWelded} or {@link AprilTagFields#k2026RebuiltAndymark}.
   */
  public Field2026(AprilTagFields variant) {
    this(AprilTagFieldLayout.loadField(variant));
  }

  /**
   * Builds the 2026 REBUILT field's collision geometry directly from {@code layout}.
   *
   * @param layout AprilTag field layout to derive the field boundary and HUB/TOWER collision
   *               geometry from.
   */
  public Field2026(AprilTagFieldLayout layout) {
    this.fieldLength = layout.getFieldLength();
    this.fieldWidth = layout.getFieldWidth();

    this.blueHub = hubElementFromTags(layout, BLUE_HUB_TAG_IDS);
    this.redHub = hubElementFromTags(layout, RED_HUB_TAG_IDS);
    this.blueTowerUprights = towerUprightsFromTags(layout, BLUE_TOWER_WALL_TAG_IDS, fieldLength);
    this.redTowerUprights = towerUprightsFromTags(layout, RED_TOWER_WALL_TAG_IDS, fieldLength);

    this.blueSouthTrenchGate = trenchGateElement(bounds(blueHub), -1);
    this.blueNorthTrenchGate = trenchGateElement(bounds(blueHub), 1);
    this.redSouthTrenchGate = trenchGateElement(bounds(redHub), -1);
    this.redNorthTrenchGate = trenchGateElement(bounds(redHub), 1);

    this.westAllianceWall = FieldLayout.rectangle(0, fieldWidth / 2.0, WALL_THICKNESS, fieldWidth);
    this.eastAllianceWall = FieldLayout.rectangle(fieldLength, fieldWidth / 2.0, WALL_THICKNESS, fieldWidth);
    this.southGuardrail = FieldLayout.rectangle(fieldLength / 2.0, 0, fieldLength, WALL_THICKNESS);
    this.northGuardrail = FieldLayout.rectangle(fieldLength / 2.0, fieldWidth, fieldLength, WALL_THICKNESS);

    elements = new ArrayList<>(List.of(
        westAllianceWall, eastAllianceWall, southGuardrail, northGuardrail, blueHub, redHub,
        blueSouthTrenchGate, blueNorthTrenchGate, redSouthTrenchGate, redNorthTrenchGate));
    elements.addAll(List.of(blueTowerUprights));
    elements.addAll(List.of(redTowerUprights));
  }

  @Override
  public void populateWorld(World<Body> world) {
    for (Element element : elements) {
      world.addBody(element.toBody());
    }
  }

  /** Full field length (X axis), alliance wall to alliance wall, in meters. */
  public double getFieldLength() {
    return fieldLength;
  }

  /** Full field width (Y axis), guardrail to guardrail, in meters. */
  public double getFieldWidth() {
    return fieldWidth;
  }

  /** The blue alliance's HUB collision element (an octagon derived from its 8 AprilTags). */
  public Element getBlueHub() {
    return blueHub;
  }

  /** The red alliance's HUB collision element (an octagon derived from its 8 AprilTags). */
  public Element getRedHub() {
    return redHub;
  }

  /**
   * The blue alliance's TOWER UPRIGHT collision elements (2, nominal cross-section, centered on
   * their derived mount position). Robots can pass beneath/between the TOWER otherwise.
   */
  public Element[] getBlueTowerUprights() {
    return blueTowerUprights.clone();
  }

  /**
   * The red alliance's TOWER UPRIGHT collision elements (2, nominal cross-section, centered on
   * their derived mount position). Robots can pass beneath/between the TOWER otherwise.
   */
  public Element[] getRedTowerUprights() {
    return redTowerUprights.clone();
  }

  /** The blue alliance's south (Y=0 guardrail) TRENCH GATE collision element. */
  public Element getBlueSouthTrenchGate() {
    return blueSouthTrenchGate;
  }

  /** The blue alliance's north (Y=field width guardrail) TRENCH GATE collision element. */
  public Element getBlueNorthTrenchGate() {
    return blueNorthTrenchGate;
  }

  /** The red alliance's south (Y=0 guardrail) TRENCH GATE collision element. */
  public Element getRedSouthTrenchGate() {
    return redSouthTrenchGate;
  }

  /** The red alliance's north (Y=field width guardrail) TRENCH GATE collision element. */
  public Element getRedNorthTrenchGate() {
    return redNorthTrenchGate;
  }

  /** The blue (X=0) alliance wall collision element. */
  public Element getWestAllianceWall() {
    return westAllianceWall;
  }

  /** The red (far, X=field length) alliance wall collision element. */
  public Element getEastAllianceWall() {
    return eastAllianceWall;
  }

  /** The Y=0 guardrail collision element. */
  public Element getSouthGuardrail() {
    return southGuardrail;
  }

  /** The Y=field width guardrail collision element. */
  public Element getNorthGuardrail() {
    return northGuardrail;
  }

  /**
   * Builds a HUB's actual octagonal footprint from its 8 AprilTags' positions -- since each tag
   * sits centered on one of the HUB's 4 faces, their positions directly trace its real outer
   * boundary, wound counter-clockwise around their centroid as {@link FieldLayout.Element} requires.
   */
  private static Element hubElementFromTags(AprilTagFieldLayout layout, int[] tagIds) {
    List<Translation2d> points = tagTranslations(layout, tagIds);

    double centerX = points.stream().mapToDouble(Translation2d::getX).average().orElseThrow();
    double centerY = points.stream().mapToDouble(Translation2d::getY).average().orElseThrow();
    points.sort(Comparator.comparingDouble(p -> Math.atan2(p.getY() - centerY, p.getX() - centerX)));

    return new Element(points.toArray(new Translation2d[0]));
  }

  /**
   * Builds a TOWER's two UPRIGHT collision boxes (nominal {@link #UPRIGHT_DEPTH}/
   * {@link #UPRIGHT_THICKNESS} cross-section, {@link #UPRIGHT_SPACING} apart center-to-center),
   * positioned from its (mount-only) AprilTags. The tags are mounted essentially at the alliance
   * wall itself, so the UPRIGHTS are centered a half-{@link #TOWER_DEPTH} inward from that mount
   * position (mirroring the TOWER's near face against the wall) rather than at the mount position
   * itself, which would place them outside the field, through the wall.
   */
  private static Element[] towerUprightsFromTags(AprilTagFieldLayout layout, int[] tagIds, double fieldLength) {
    List<Translation2d> points = tagTranslations(layout, tagIds);
    double mountX = points.stream().mapToDouble(Translation2d::getX).average().orElseThrow();
    double centerY = points.stream().mapToDouble(Translation2d::getY).average().orElseThrow();
    boolean nearWestWall = mountX < fieldLength / 2.0;
    double centerX = nearWestWall ? mountX + TOWER_DEPTH / 2.0 : mountX - TOWER_DEPTH / 2.0;
    return new Element[] {
        FieldLayout.rectangle(centerX, centerY - UPRIGHT_SPACING / 2.0, UPRIGHT_DEPTH, UPRIGHT_THICKNESS),
        FieldLayout.rectangle(centerX, centerY + UPRIGHT_SPACING / 2.0, UPRIGHT_DEPTH, UPRIGHT_THICKNESS)
    };
  }

  /**
   * Builds one TRENCH's GATE collision box: {@link #TRENCH_GATE_WIDTH} x {@link #TRENCH_GATE_DEPTH},
   * X-centered on the given HUB (matching the manual's "attached at the middle of the BUMP
   * length-wise", since the BUMP shares the HUB's row), and Y-positioned just outside the BUMP's
   * nominal {@link #BUMP_WIDTH} footprint on the given guardrail's side of that HUB -- i.e. within
   * the TRENCH's own space, immediately next to the (unmodeled) BUMP it connects to.
   *
   * @param hubBounds            {minX, maxX, minY, maxY} of the HUB this TRENCH's BUMP is next to.
   * @param towardGuardrailSign  {@code -1} for the south (Y=0) guardrail, {@code +1} for the north.
   */
  private static Element trenchGateElement(double[] hubBounds, double towardGuardrailSign) {
    double hubCenterX = (hubBounds[0] + hubBounds[1]) / 2.0;
    double hubEdgeY = towardGuardrailSign < 0 ? hubBounds[2] : hubBounds[3];
    double bumpOuterEdgeY = hubEdgeY + towardGuardrailSign * BUMP_WIDTH;
    double gateCenterY = bumpOuterEdgeY + towardGuardrailSign * (TRENCH_GATE_DEPTH / 2.0);
    return FieldLayout.rectangle(hubCenterX, gateCenterY, TRENCH_GATE_WIDTH, TRENCH_GATE_DEPTH);
  }

  /** Returns {minX, maxX, minY, maxY} over an element's vertices, in field-relative meters. */
  private static double[] bounds(Element element) {
    double minX = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;
    for (Translation2d v : element.getVertices()) {
      minX = Math.min(minX, v.getX());
      maxX = Math.max(maxX, v.getX());
      minY = Math.min(minY, v.getY());
      maxY = Math.max(maxY, v.getY());
    }
    return new double[] {minX, maxX, minY, maxY};
  }

  private static List<Translation2d> tagTranslations(AprilTagFieldLayout layout, int[] tagIds) {
    List<Translation2d> points = new ArrayList<>();
    for (int id : tagIds) {
      points.add(layout.getTagPose(id)
          .orElseThrow(() -> new IllegalStateException(
              "AprilTag field layout is missing tag " + id + " required to build Field2026"))
          .toPose2d().getTranslation());
    }
    return points;
  }
}
