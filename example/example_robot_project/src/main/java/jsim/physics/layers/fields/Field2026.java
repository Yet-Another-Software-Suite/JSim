package jsim.physics.layers.fields;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.List;

import org.dyn4j.dynamics.Body;
import org.dyn4j.world.World;

import jsim.physics.layers.FieldLayout;

/**
 * Static field geometry for the 2026 FIRST Robotics Competition game, REBUILT.
 *
 * <p>Every collision element below is a hardcoded, field-relative vertex list (in meters). These
 * were computed once from the official welded-field AprilTag layout plus the Game Manual's known
 * structure dimensions, then baked in directly -- this class has no runtime dependency on {@code
 * AprilTagFieldLayout}. The welded and AndyMark field variants only differ in AprilTag mounting
 * position, not in the physical structures modeled here (walls, HUB, TOWER uprights, TRENCH
 * gates), so a single hardcoded layout covers both.
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
 * ~1in DEPOT barrier are omitted. BUMPS themselves are also omitted since robots drive over them;
 * each TRENCH GATE below is still positioned immediately outside its BUMP's footprint, matching
 * where that gate's hardware actually attaches.
 *
 * <p>Field coordinate convention (matches WPILib): origin at the blue ALLIANCE WALL corner at the
 * Y=0 guardrail, +X towards the red ALLIANCE WALL, +Y across the width of the FIELD.
 */
public class Field2026 implements FieldLayout {

  /** Full field length (X axis), alliance wall to alliance wall, in meters. */
  private static final double FIELD_LENGTH = 16.541;

  /** Full field width (Y axis), guardrail to guardrail, in meters. */
  private static final double FIELD_WIDTH = 8.069;

  /** Thickness of the modeled boundary walls (guardrails and alliance walls), in meters. */
  private static final double WALL_THICKNESS = 0.0508;

  /** 2in-thick boundary wall along the blue (X=0) alliance wall, spanning the full field width. */
  private static final Element WEST_ALLIANCE_WALL =
      FieldLayout.rectangle(0, FIELD_WIDTH / 2.0, WALL_THICKNESS, FIELD_WIDTH);

  /** 2in-thick boundary wall along the red (X=FIELD_LENGTH) alliance wall, spanning the full field width. */
  private static final Element EAST_ALLIANCE_WALL =
      FieldLayout.rectangle(FIELD_LENGTH, FIELD_WIDTH / 2.0, WALL_THICKNESS, FIELD_WIDTH);

  /** 2in-thick boundary wall along the Y=0 guardrail, spanning the full field length. */
  private static final Element SOUTH_GUARDRAIL =
      FieldLayout.rectangle(FIELD_LENGTH / 2.0, 0, FIELD_LENGTH, WALL_THICKNESS);

  /** 2in-thick boundary wall along the Y=FIELD_WIDTH guardrail, spanning the full field length. */
  private static final Element NORTH_GUARDRAIL =
      FieldLayout.rectangle(FIELD_LENGTH / 2.0, FIELD_WIDTH, FIELD_LENGTH, WALL_THICKNESS);

  /** Blue alliance's HUB: a 47in square base (Section 5.11). */
  private static final Element BLUE_HUB = new Element(
      new Translation2d(4.0219, 3.4377),
      new Translation2d(5.2157, 3.4377),
      new Translation2d(5.2157, 4.6315),
      new Translation2d(4.0219, 4.6315));

  /** Red alliance's HUB: a 47in square base (Section 5.11). */
  private static final Element RED_HUB = new Element(
      new Translation2d(11.3119, 3.4377),
      new Translation2d(12.5057, 3.4377),
      new Translation2d(12.5057, 4.6315),
      new Translation2d(11.3119, 4.6315));

  /** Blue alliance's TOWER, south (nearer Y=0) UPRIGHT (Section 5.8). */
  private static final Element BLUE_TOWER_UPRIGHT_SOUTH = new Element(
      new Translation2d(1.0607, 3.5139),
      new Translation2d(1.1496, 3.5139),
      new Translation2d(1.1496, 3.5520),
      new Translation2d(1.0607, 3.5520));

  /** Blue alliance's TOWER, north (nearer Y=FIELD_WIDTH) UPRIGHT (Section 5.8). */
  private static final Element BLUE_TOWER_UPRIGHT_NORTH = new Element(
      new Translation2d(1.0607, 4.3712),
      new Translation2d(1.1496, 4.3712),
      new Translation2d(1.1496, 4.4093),
      new Translation2d(1.0607, 4.4093));

  /** Red alliance's TOWER, south (nearer Y=0) UPRIGHT (Section 5.8). */
  private static final Element RED_TOWER_UPRIGHT_SOUTH = new Element(
      new Translation2d(15.3914, 3.6600),
      new Translation2d(15.4803, 3.6600),
      new Translation2d(15.4803, 3.6981),
      new Translation2d(15.3914, 3.6981));

  /** Red alliance's TOWER, north (nearer Y=FIELD_WIDTH) UPRIGHT (Section 5.8). */
  private static final Element RED_TOWER_UPRIGHT_NORTH = new Element(
      new Translation2d(15.3914, 4.5172),
      new Translation2d(15.4803, 4.5172),
      new Translation2d(15.4803, 4.5553),
      new Translation2d(15.3914, 4.5553));

  /** Blue alliance's south (Y=0 guardrail side) TRENCH GATE (Section 5.6). */
  private static final Element BLUE_SOUTH_TRENCH_GATE = new Element(
      new Translation2d(4.0549, 1.2141),
      new Translation2d(5.1826, 1.2141),
      new Translation2d(5.1826, 1.6030),
      new Translation2d(4.0549, 1.6030));

  /** Blue alliance's north (Y=FIELD_WIDTH guardrail side) TRENCH GATE (Section 5.6). */
  private static final Element BLUE_NORTH_TRENCH_GATE = new Element(
      new Translation2d(4.0549, 6.4663),
      new Translation2d(5.1826, 6.4663),
      new Translation2d(5.1826, 6.8552),
      new Translation2d(4.0549, 6.8552));

  /** Red alliance's south (Y=0 guardrail side) TRENCH GATE (Section 5.6). */
  private static final Element RED_SOUTH_TRENCH_GATE = new Element(
      new Translation2d(11.3449, 1.2141),
      new Translation2d(12.4726, 1.2141),
      new Translation2d(12.4726, 1.6030),
      new Translation2d(11.3449, 1.6030));

  /** Red alliance's north (Y=FIELD_WIDTH guardrail side) TRENCH GATE (Section 5.6). */
  private static final Element RED_NORTH_TRENCH_GATE = new Element(
      new Translation2d(11.3449, 6.4663),
      new Translation2d(12.4726, 6.4663),
      new Translation2d(12.4726, 6.8552),
      new Translation2d(11.3449, 6.8552));

  private static final List<Element> ELEMENTS = List.of(
      WEST_ALLIANCE_WALL, EAST_ALLIANCE_WALL, SOUTH_GUARDRAIL, NORTH_GUARDRAIL,
      BLUE_HUB, RED_HUB,
      BLUE_TOWER_UPRIGHT_SOUTH, BLUE_TOWER_UPRIGHT_NORTH, RED_TOWER_UPRIGHT_SOUTH, RED_TOWER_UPRIGHT_NORTH,
      BLUE_SOUTH_TRENCH_GATE, BLUE_NORTH_TRENCH_GATE, RED_SOUTH_TRENCH_GATE, RED_NORTH_TRENCH_GATE);

  /** SmartDashboard field visualization; replaceable by callers (e.g. {@code field.field = drive.getField2d()}). */
  public Field2d field = new Field2d();

  @Override
  public void populateWorld(World<Body> world) {
    int i = 0;
    for (Element element : ELEMENTS) {
      world.addBody(element.toBody());
      field.getObject(String.valueOf(i++)).setPoses(element.getPoses());
    }
  }

  /** Full field length (X axis), alliance wall to alliance wall, in meters. */
  public double getFieldLength() {
    return FIELD_LENGTH;
  }

  /** Full field width (Y axis), guardrail to guardrail, in meters. */
  public double getFieldWidth() {
    return FIELD_WIDTH;
  }

  /** The blue alliance's HUB collision element (a 47in square, 4 corners). */
  public Element getBlueHub() {
    return BLUE_HUB;
  }

  /** The red alliance's HUB collision element (a 47in square, 4 corners). */
  public Element getRedHub() {
    return RED_HUB;
  }

  /**
   * The blue alliance's TOWER UPRIGHT collision elements (south, then north).
   * Robots can pass beneath/between the TOWER otherwise.
   */
  public Element[] getBlueTowerUprights() {
    return new Element[] {BLUE_TOWER_UPRIGHT_SOUTH, BLUE_TOWER_UPRIGHT_NORTH};
  }

  /**
   * The red alliance's TOWER UPRIGHT collision elements (south, then north).
   * Robots can pass beneath/between the TOWER otherwise.
   */
  public Element[] getRedTowerUprights() {
    return new Element[] {RED_TOWER_UPRIGHT_SOUTH, RED_TOWER_UPRIGHT_NORTH};
  }

  /** The blue alliance's south (Y=0 guardrail) TRENCH GATE collision element. */
  public Element getBlueSouthTrenchGate() {
    return BLUE_SOUTH_TRENCH_GATE;
  }

  /** The blue alliance's north (Y=field width guardrail) TRENCH GATE collision element. */
  public Element getBlueNorthTrenchGate() {
    return BLUE_NORTH_TRENCH_GATE;
  }

  /** The red alliance's south (Y=0 guardrail) TRENCH GATE collision element. */
  public Element getRedSouthTrenchGate() {
    return RED_SOUTH_TRENCH_GATE;
  }

  /** The red alliance's north (Y=field width guardrail) TRENCH GATE collision element. */
  public Element getRedNorthTrenchGate() {
    return RED_NORTH_TRENCH_GATE;
  }

  /** The blue (X=0) alliance wall collision element. */
  public Element getWestAllianceWall() {
    return WEST_ALLIANCE_WALL;
  }

  /** The red (far, X=field length) alliance wall collision element. */
  public Element getEastAllianceWall() {
    return EAST_ALLIANCE_WALL;
  }

  /** The Y=0 guardrail collision element. */
  public Element getSouthGuardrail() {
    return SOUTH_GUARDRAIL;
  }

  /** The Y=field width guardrail collision element. */
  public Element getNorthGuardrail() {
    return NORTH_GUARDRAIL;
  }
}
