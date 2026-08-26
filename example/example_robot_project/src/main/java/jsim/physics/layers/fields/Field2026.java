package jsim.physics.layers.fields;

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

  /** The HUB's square base footprint (both axes), per Section 5.11. */
  private static final double HUB_WIDTH = 1.1938;

  /** Blue alliance's HUB: a {@link #HUB_WIDTH} square base (Section 5.11). */
  private static final Element BLUE_HUB =
      FieldLayout.rectangle(4.6188, 4.0346, HUB_WIDTH, HUB_WIDTH);

  /** Red alliance's HUB: a {@link #HUB_WIDTH} square base (Section 5.11). */
  private static final Element RED_HUB =
      FieldLayout.rectangle(11.9088, 4.0346, HUB_WIDTH, HUB_WIDTH);

  /** Each TOWER UPRIGHT's X-axis cross-section (~3.5in), per Section 5.8. */
  private static final double UPRIGHT_DEPTH = 0.0889;

  /** Each TOWER UPRIGHT's Y-axis cross-section (~1.5in), per Section 5.8. */
  private static final double UPRIGHT_THICKNESS = 0.0381;

  /** Blue alliance's TOWER, south (nearer Y=0) UPRIGHT (Section 5.8). */
  private static final Element BLUE_TOWER_UPRIGHT_SOUTH =
      FieldLayout.rectangle(1.10515, 3.53295, UPRIGHT_DEPTH, UPRIGHT_THICKNESS);

  /** Blue alliance's TOWER, north (nearer Y=FIELD_WIDTH) UPRIGHT (Section 5.8). */
  private static final Element BLUE_TOWER_UPRIGHT_NORTH =
      FieldLayout.rectangle(1.10515, 4.39025, UPRIGHT_DEPTH, UPRIGHT_THICKNESS);

  /** Red alliance's TOWER, south (nearer Y=0) UPRIGHT (Section 5.8). */
  private static final Element RED_TOWER_UPRIGHT_SOUTH =
      FieldLayout.rectangle(15.43585, 3.67905, UPRIGHT_DEPTH, UPRIGHT_THICKNESS);

  /** Red alliance's TOWER, north (nearer Y=FIELD_WIDTH) UPRIGHT (Section 5.8). */
  private static final Element RED_TOWER_UPRIGHT_NORTH =
      FieldLayout.rectangle(15.43585, 4.53625, UPRIGHT_DEPTH, UPRIGHT_THICKNESS);

  /**
   * Each TRENCH GATE's X-axis extent (~44.4in), per Section 5.6. Note: the manual describes the
   * gate's dimensions as a 15.31in width parallel to the guardrail and a 44.4in depth perpendicular
   * to it -- these two constants preserve the field's already-baked axis mapping rather than the
   * manual's own width/depth labels (which appear swapped from that mapping); see the class javadoc.
   */
  private static final double TRENCH_GATE_WIDTH = 1.1277;

  /** Each TRENCH GATE's Y-axis extent (~15.31in), per Section 5.6. See {@link #TRENCH_GATE_WIDTH}. */
  private static final double TRENCH_GATE_DEPTH = 0.3889;

  /** Blue alliance's south (Y=0 guardrail side) TRENCH GATE (Section 5.6). */
  private static final Element BLUE_SOUTH_TRENCH_GATE =
      FieldLayout.rectangle(4.61875, 1.40855, TRENCH_GATE_WIDTH, TRENCH_GATE_DEPTH);

  /** Blue alliance's north (Y=FIELD_WIDTH guardrail side) TRENCH GATE (Section 5.6). */
  private static final Element BLUE_NORTH_TRENCH_GATE =
      FieldLayout.rectangle(4.61875, 6.66075, TRENCH_GATE_WIDTH, TRENCH_GATE_DEPTH);

  /** Red alliance's south (Y=0 guardrail side) TRENCH GATE (Section 5.6). */
  private static final Element RED_SOUTH_TRENCH_GATE =
      FieldLayout.rectangle(11.90875, 1.40855, TRENCH_GATE_WIDTH, TRENCH_GATE_DEPTH);

  /** Red alliance's north (Y=FIELD_WIDTH guardrail side) TRENCH GATE (Section 5.6). */
  private static final Element RED_NORTH_TRENCH_GATE =
      FieldLayout.rectangle(11.90875, 6.66075, TRENCH_GATE_WIDTH, TRENCH_GATE_DEPTH);

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
