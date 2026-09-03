package jsim.physics.layers.fields;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.ArrayList;
import java.util.List;

import jsim.physics.layers.FuelLayer;
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
 * <h3>Drivetrain elements vs. game-piece elements</h3>
 *
 * <p>{@link #populateWorld(World)} injects only the structures a <i>robot</i> can hit into the 2D
 * dyn4j world ({@link #getDriveObstacles()}) -- thin items like tape lines, AprilTag panels, and
 * the ~1in DEPOT barrier are omitted, as are BUMPS (robots drive over them) and TRENCH BARS
 * (robots drive under them).
 *
 * <p>A FUEL ball, however, very much interacts with those: it rolls up and over BUMPS, rattles
 * off TRENCH BARS, and rests on top of TRENCH gates. So every element also carries a vertical
 * extent, and {@link #getGamePieceObstacles()} returns the larger set that a 3D game-piece layer
 * (see {@link FuelLayer}) collides against. Heights not stated by the manual are modeled to the
 * nearest sensible value and noted as such below; they only affect game-piece simulation, never
 * drivetrain collisions.
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

  /**
   * Modeled height of the guardrail plus the netting above it (~78in). Only used by game-piece
   * layers, to keep launched FUEL inside the FIELD.
   */
  private static final double PERIMETER_HEIGHT = 1.9812;

  /** 2in-thick boundary wall along the blue (X=0) alliance wall, spanning the full field width. */
  private static final Element WEST_ALLIANCE_WALL =
      FieldLayout.box(0, FIELD_WIDTH / 2.0, WALL_THICKNESS, FIELD_WIDTH, 0.0, PERIMETER_HEIGHT);

  /** 2in-thick boundary wall along the red (X=FIELD_LENGTH) alliance wall, spanning the full field width. */
  private static final Element EAST_ALLIANCE_WALL =
      FieldLayout.box(FIELD_LENGTH, FIELD_WIDTH / 2.0, WALL_THICKNESS, FIELD_WIDTH, 0.0, PERIMETER_HEIGHT);

  /** 2in-thick boundary wall along the Y=0 guardrail, spanning the full field length. */
  private static final Element SOUTH_GUARDRAIL =
      FieldLayout.box(FIELD_LENGTH / 2.0, 0, FIELD_LENGTH, WALL_THICKNESS, 0.0, PERIMETER_HEIGHT);

  /** 2in-thick boundary wall along the Y=FIELD_WIDTH guardrail, spanning the full field length. */
  private static final Element NORTH_GUARDRAIL =
      FieldLayout.box(FIELD_LENGTH / 2.0, FIELD_WIDTH, FIELD_LENGTH, WALL_THICKNESS, 0.0, PERIMETER_HEIGHT);

  /** The HUB's square base footprint (both axes), per Section 5.11. */
  private static final double HUB_WIDTH = 1.1938;

  /** X coordinate of the blue HUB's center, in meters. */
  private static final double BLUE_HUB_CENTER_X = 4.6188;

  /** X coordinate of the red HUB's center, in meters. */
  private static final double RED_HUB_CENTER_X = 11.9088;

  /** Y coordinate of both HUB centers, in meters. */
  private static final double HUB_CENTER_Y = 4.0346;

  /**
   * Height of the HUB's solid outer wall, in meters -- just below {@link #HUB_ENTRY_HEIGHT}, so
   * a ball arriving above the wall drops into the goal instead of bouncing off it.
   */
  private static final double HUB_SIDE_HEIGHT = 1.73;

  /** Height of the HUB's open goal above the carpet (Section 5.11), in meters. */
  private static final double HUB_ENTRY_HEIGHT = 1.83;

  /** Radius of the HUB's open goal about the HUB center, in meters. */
  private static final double HUB_ENTRY_RADIUS = 0.56;

  /** Height at which scored FUEL is returned to the FIELD from the HUB, in meters. */
  private static final double HUB_EXIT_HEIGHT = 0.89;

  /** Gap between the HUB's field-center face and the point scored FUEL re-enters play, in meters. */
  private static final double HUB_EXIT_CLEARANCE = 0.084;

  /** Blue alliance's HUB: a {@link #HUB_WIDTH} square base (Section 5.11). */
  private static final Element BLUE_HUB =
      FieldLayout.box(BLUE_HUB_CENTER_X, HUB_CENTER_Y, HUB_WIDTH, HUB_WIDTH, 0.0, HUB_SIDE_HEIGHT);

  /** Red alliance's HUB: a {@link #HUB_WIDTH} square base (Section 5.11). */
  private static final Element RED_HUB =
      FieldLayout.box(RED_HUB_CENTER_X, HUB_CENTER_Y, HUB_WIDTH, HUB_WIDTH, 0.0, HUB_SIDE_HEIGHT);

  /** Y coordinate of both HUBs' Y=0-side face, in meters. */
  private static final double HUB_SOUTH_Y = HUB_CENTER_Y - HUB_WIDTH / 2.0;

  /** Y coordinate of both HUBs' Y=FIELD_WIDTH-side face, in meters. */
  private static final double HUB_NORTH_Y = HUB_CENTER_Y + HUB_WIDTH / 2.0;

  /** Where scored FUEL re-enters play from the blue HUB, ejected towards the FIELD center. */
  private static final Translation3d BLUE_HUB_EXIT = new Translation3d(
      BLUE_HUB_CENTER_X + HUB_WIDTH / 2.0 + HUB_EXIT_CLEARANCE, HUB_CENTER_Y, HUB_EXIT_HEIGHT);

  /** Where scored FUEL re-enters play from the red HUB, ejected towards the FIELD center. */
  private static final Translation3d RED_HUB_EXIT = new Translation3d(
      RED_HUB_CENTER_X - HUB_WIDTH / 2.0 - HUB_EXIT_CLEARANCE, HUB_CENTER_Y, HUB_EXIT_HEIGHT);

  /** Height of the bottom edge of each HUB's backing net above the carpet, in meters. */
  private static final double NET_BOTTOM_HEIGHT = 1.5;

  /** Height of the top edge of each HUB's backing net above the carpet, in meters. */
  private static final double NET_TOP_HEIGHT = 3.057;

  /** Each HUB net's extent along the Y axis, in meters. */
  private static final double NET_DEPTH = 1.484;

  /** Modeled thickness of each HUB net along the X axis (1in), in meters. */
  private static final double NET_THICKNESS = 0.0254;

  /** Distance from a HUB's center to its net, measured towards the FIELD center, in meters. */
  private static final double NET_OFFSET = HUB_WIDTH / 2.0 + 0.261;

  /** The blue HUB's backing net, hanging above and towards the FIELD center from the HUB. */
  private static final Element BLUE_HUB_NET = FieldLayout.box(
      BLUE_HUB_CENTER_X + NET_OFFSET, HUB_CENTER_Y, NET_THICKNESS, NET_DEPTH,
      NET_BOTTOM_HEIGHT, NET_TOP_HEIGHT);

  /** The red HUB's backing net, hanging above and towards the FIELD center from the HUB. */
  private static final Element RED_HUB_NET = FieldLayout.box(
      RED_HUB_CENTER_X - NET_OFFSET, HUB_CENTER_Y, NET_THICKNESS, NET_DEPTH,
      NET_BOTTOM_HEIGHT, NET_TOP_HEIGHT);

  /** Each TOWER UPRIGHT's X-axis cross-section (~3.5in), per Section 5.8. */
  private static final double UPRIGHT_DEPTH = 0.0889;

  /** Each TOWER UPRIGHT's Y-axis cross-section (~1.5in), per Section 5.8. */
  private static final double UPRIGHT_THICKNESS = 0.0381;
  private static final double UPRIGHT_OFFSET = Inches.of(7).in(Meters);

  /** Modeled height of each TOWER UPRIGHT (~72in), used only for game-piece collisions. */
  private static final double UPRIGHT_HEIGHT = 1.8288;

  /** Blue alliance's TOWER, south (nearer Y=0) UPRIGHT (Section 5.8). */
  private static final Element BLUE_TOWER_UPRIGHT_SOUTH = FieldLayout.box(
      1.10515, 3.53295 - UPRIGHT_OFFSET, UPRIGHT_DEPTH, UPRIGHT_THICKNESS, 0.0, UPRIGHT_HEIGHT);

  /** Blue alliance's TOWER, north (nearer Y=FIELD_WIDTH) UPRIGHT (Section 5.8). */
  private static final Element BLUE_TOWER_UPRIGHT_NORTH = FieldLayout.box(
      1.10515, 4.39025 - UPRIGHT_OFFSET, UPRIGHT_DEPTH, UPRIGHT_THICKNESS, 0.0, UPRIGHT_HEIGHT);

  /** Red alliance's TOWER, south (nearer Y=0) UPRIGHT (Section 5.8). */
  private static final Element RED_TOWER_UPRIGHT_SOUTH = FieldLayout.box(
      15.43585, 3.67905 + UPRIGHT_OFFSET, UPRIGHT_DEPTH, UPRIGHT_THICKNESS, 0.0, UPRIGHT_HEIGHT);

  /** Red alliance's TOWER, north (nearer Y=FIELD_WIDTH) UPRIGHT (Section 5.8). */
  private static final Element RED_TOWER_UPRIGHT_NORTH = FieldLayout.box(
      15.43585, 4.53625 + UPRIGHT_OFFSET, UPRIGHT_DEPTH, UPRIGHT_THICKNESS, 0.0, UPRIGHT_HEIGHT);

  /**
   * Each TRENCH GATE's X-axis extent (~44.4in), per Section 5.6. Note: the manual describes the
   * gate's dimensions as a 15.31in width parallel to the guardrail and a 44.4in depth perpendicular
   * to it -- these two constants preserve the field's already-baked axis mapping rather than the
   * manual's own width/depth labels (which appear swapped from that mapping); see the class javadoc.
   */
  private static final double TRENCH_GATE_WIDTH = 1.1277;

  /** Each TRENCH GATE's Y-axis extent (~15.31in), per Section 5.6. See {@link #TRENCH_GATE_WIDTH}. */
  private static final double TRENCH_GATE_DEPTH = 0.3889;

  /** Height of the TRENCH structure above the carpet (~22.2in), per Section 5.6. */
  private static final double TRENCH_HEIGHT = 0.565;

  /** X coordinate of the center of both blue TRENCH GATEs, in meters. */
  private static final double BLUE_TRENCH_CENTER_X = 4.61875;

  /** X coordinate of the center of both red TRENCH GATEs, in meters. */
  private static final double RED_TRENCH_CENTER_X = 11.90875;

  /** Y coordinate of the center of both south (Y=0 side) TRENCH GATEs, in meters. */
  private static final double SOUTH_TRENCH_CENTER_Y = 1.40855;

  /** Y coordinate of the center of both north (Y=FIELD_WIDTH side) TRENCH GATEs, in meters. */
  private static final double NORTH_TRENCH_CENTER_Y = 6.66075;

  /** Y coordinate of the south TRENCH GATEs' inner (field-center-facing) face, in meters. */
  private static final double SOUTH_TRENCH_INNER_Y = SOUTH_TRENCH_CENTER_Y + TRENCH_GATE_DEPTH / 2.0;

  /** Y coordinate of the north TRENCH GATEs' inner (field-center-facing) face, in meters. */
  private static final double NORTH_TRENCH_INNER_Y = NORTH_TRENCH_CENTER_Y - TRENCH_GATE_DEPTH / 2.0;

  /** Blue alliance's south (Y=0 guardrail side) TRENCH GATE (Section 5.6). */
  private static final Element BLUE_SOUTH_TRENCH_GATE = FieldLayout.box(
      BLUE_TRENCH_CENTER_X, SOUTH_TRENCH_CENTER_Y, TRENCH_GATE_WIDTH, TRENCH_GATE_DEPTH,
      0.0, TRENCH_HEIGHT);

  /** Blue alliance's north (Y=FIELD_WIDTH guardrail side) TRENCH GATE (Section 5.6). */
  private static final Element BLUE_NORTH_TRENCH_GATE = FieldLayout.box(
      BLUE_TRENCH_CENTER_X, NORTH_TRENCH_CENTER_Y, TRENCH_GATE_WIDTH, TRENCH_GATE_DEPTH,
      0.0, TRENCH_HEIGHT);

  /** Red alliance's south (Y=0 guardrail side) TRENCH GATE (Section 5.6). */
  private static final Element RED_SOUTH_TRENCH_GATE = FieldLayout.box(
      RED_TRENCH_CENTER_X, SOUTH_TRENCH_CENTER_Y, TRENCH_GATE_WIDTH, TRENCH_GATE_DEPTH,
      0.0, TRENCH_HEIGHT);

  /** Red alliance's north (Y=FIELD_WIDTH guardrail side) TRENCH GATE (Section 5.6). */
  private static final Element RED_NORTH_TRENCH_GATE = FieldLayout.box(
      RED_TRENCH_CENTER_X, NORTH_TRENCH_CENTER_Y, TRENCH_GATE_WIDTH, TRENCH_GATE_DEPTH,
      0.0, TRENCH_HEIGHT);

  /** Each TRENCH BAR's X-axis cross-section (~6in), per Section 5.6. */
  private static final double TRENCH_BAR_WIDTH = 0.152;

  /** How far each TRENCH BAR stands above the TRENCH structure (~4in), per Section 5.6. */
  private static final double TRENCH_BAR_HEIGHT = 0.102;

  /**
   * Blue alliance's south TRENCH BAR: spans from the Y=0 guardrail to its TRENCH GATE, sitting on
   * top of the TRENCH. Robots drive under it, so it is game-piece-only geometry.
   */
  private static final Element BLUE_SOUTH_TRENCH_BAR = FieldLayout.box(
      BLUE_TRENCH_CENTER_X, SOUTH_TRENCH_INNER_Y / 2.0, TRENCH_BAR_WIDTH, SOUTH_TRENCH_INNER_Y,
      TRENCH_HEIGHT, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT);

  /** Blue alliance's north TRENCH BAR. See {@link #BLUE_SOUTH_TRENCH_BAR}. */
  private static final Element BLUE_NORTH_TRENCH_BAR = FieldLayout.box(
      BLUE_TRENCH_CENTER_X, (NORTH_TRENCH_INNER_Y + FIELD_WIDTH) / 2.0,
      TRENCH_BAR_WIDTH, FIELD_WIDTH - NORTH_TRENCH_INNER_Y,
      TRENCH_HEIGHT, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT);

  /** Red alliance's south TRENCH BAR. See {@link #BLUE_SOUTH_TRENCH_BAR}. */
  private static final Element RED_SOUTH_TRENCH_BAR = FieldLayout.box(
      RED_TRENCH_CENTER_X, SOUTH_TRENCH_INNER_Y / 2.0, TRENCH_BAR_WIDTH, SOUTH_TRENCH_INNER_Y,
      TRENCH_HEIGHT, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT);

  /** Red alliance's north TRENCH BAR. See {@link #BLUE_SOUTH_TRENCH_BAR}. */
  private static final Element RED_NORTH_TRENCH_BAR = FieldLayout.box(
      RED_TRENCH_CENTER_X, (NORTH_TRENCH_INNER_Y + FIELD_WIDTH) / 2.0,
      TRENCH_BAR_WIDTH, FIELD_WIDTH - NORTH_TRENCH_INNER_Y,
      TRENCH_HEIGHT, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT);

  /** Height of each BUMP's crest above the carpet (~6.5in), per Section 5.5. */
  private static final double BUMP_HEIGHT = 0.165;

  /** Horizontal run of one BUMP face, from its toe out to its crest (~24in), per Section 5.5. */
  private static final double BUMP_RUN = 0.61;

  /**
   * The four faces of the blue BUMP -- an ascending and a descending ramp in each of the two Y
   * bands the BUMP occupies (between the TRENCH and the HUB, on either side of the HUB).
   */
  private static final List<Element> BLUE_BUMP_FACES = bumpFaces(BLUE_HUB_CENTER_X);

  /** The four faces of the red BUMP. See {@link #BLUE_BUMP_FACES}. */
  private static final List<Element> RED_BUMP_FACES = bumpFaces(RED_HUB_CENTER_X);

  /** Modeled X extent of each DEPOT (~24in), where an alliance's FUEL starts the match. */
  private static final double DEPOT_WIDTH = 0.6096;

  /** Modeled Y extent of each DEPOT (~36in). */
  private static final double DEPOT_DEPTH = 0.9144;

  /** Height of the DEPOT barrier above the carpet (~1in), per Section 5.7. */
  private static final double DEPOT_BARRIER_HEIGHT = 0.0254;

  /** Y coordinate of the blue DEPOT's center, in meters. */
  private static final double BLUE_DEPOT_CENTER_Y = 5.95;

  /** Y coordinate of the red DEPOT's center -- mirrored across the FIELD's center, in meters. */
  private static final double RED_DEPOT_CENTER_Y = FIELD_WIDTH - BLUE_DEPOT_CENTER_Y;

  /** The blue DEPOT, against the blue ALLIANCE WALL. */
  private static final Element BLUE_DEPOT = FieldLayout.box(
      DEPOT_WIDTH / 2.0, BLUE_DEPOT_CENTER_Y, DEPOT_WIDTH, DEPOT_DEPTH,
      0.0, DEPOT_BARRIER_HEIGHT);

  /** The red DEPOT, against the red ALLIANCE WALL. */
  private static final Element RED_DEPOT = FieldLayout.box(
      FIELD_LENGTH - DEPOT_WIDTH / 2.0, RED_DEPOT_CENTER_Y, DEPOT_WIDTH, DEPOT_DEPTH,
      0.0, DEPOT_BARRIER_HEIGHT);

  /** Structures a robot can collide with; the only ones injected into the 2D dyn4j world. */
  private static final List<Element> ELEMENTS = List.of(
      WEST_ALLIANCE_WALL, EAST_ALLIANCE_WALL, SOUTH_GUARDRAIL, NORTH_GUARDRAIL,
      BLUE_HUB, RED_HUB,
      BLUE_TOWER_UPRIGHT_SOUTH, BLUE_TOWER_UPRIGHT_NORTH, RED_TOWER_UPRIGHT_SOUTH, RED_TOWER_UPRIGHT_NORTH,
      BLUE_SOUTH_TRENCH_GATE, BLUE_NORTH_TRENCH_GATE, RED_SOUTH_TRENCH_GATE, RED_NORTH_TRENCH_GATE);

  /** Everything a game piece can collide with: {@link #ELEMENTS} plus BUMPS and TRENCH BARS. */
  private static final List<Element> GAME_PIECE_ELEMENTS = gamePieceElements();

  /**
   * Builds the four ramp faces of one BUMP, crested at {@code crestX}. The BUMP runs across the
   * FIELD alongside its HUB but is interrupted by the HUB itself and by the two TRENCHes, leaving
   * two Y bands: one between the south TRENCH and the HUB, one between the HUB and the north
   * TRENCH. Each band gets an ascending (+X) and a descending face.
   */
  private static List<Element> bumpFaces(double crestX) {
    double southCenterY = (SOUTH_TRENCH_INNER_Y + HUB_SOUTH_Y) / 2.0;
    double southDepth = HUB_SOUTH_Y - SOUTH_TRENCH_INNER_Y;
    double northCenterY = (HUB_NORTH_Y + NORTH_TRENCH_INNER_Y) / 2.0;
    double northDepth = NORTH_TRENCH_INNER_Y - HUB_NORTH_Y;

    return List.of(
        FieldLayout.ramp(
            crestX - BUMP_RUN / 2.0, southCenterY, BUMP_RUN, southDepth, 0.0, 0.0, BUMP_HEIGHT),
        FieldLayout.ramp(
            crestX + BUMP_RUN / 2.0, southCenterY, BUMP_RUN, southDepth, 0.0, BUMP_HEIGHT, 0.0),
        FieldLayout.ramp(
            crestX - BUMP_RUN / 2.0, northCenterY, BUMP_RUN, northDepth, 0.0, 0.0, BUMP_HEIGHT),
        FieldLayout.ramp(
            crestX + BUMP_RUN / 2.0, northCenterY, BUMP_RUN, northDepth, 0.0, BUMP_HEIGHT, 0.0));
  }

  /** Assembles {@link #GAME_PIECE_ELEMENTS}. */
  private static List<Element> gamePieceElements() {
    List<Element> elements = new ArrayList<>(ELEMENTS);
    elements.add(BLUE_SOUTH_TRENCH_BAR);
    elements.add(BLUE_NORTH_TRENCH_BAR);
    elements.add(RED_SOUTH_TRENCH_BAR);
    elements.add(RED_NORTH_TRENCH_BAR);
    elements.addAll(BLUE_BUMP_FACES);
    elements.addAll(RED_BUMP_FACES);
    return List.copyOf(elements);
  }

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

  /**
   * The structures a <i>robot</i> can collide with -- exactly what {@link #populateWorld(World)}
   * injects into the 2D dyn4j world.
   */
  public List<Element> getDriveObstacles() {
    return ELEMENTS;
  }

  /**
   * The structures a <i>game piece</i> can collide with: {@link #getDriveObstacles()} plus the
   * BUMPS and TRENCH BARS that robots pass over/under but a ball does not. Each element carries
   * its vertical extent, and BUMP faces report {@code true} from
   * {@link Element#isSloped()} so a piece rolls up them.
   *
   * <p>The HUB nets ({@link #getBlueHubNet()}, {@link #getRedHubNet()}) are excluded: they are
   * modeled as thin, one-sided planes and need their own softer collision response, so a
   * game-piece layer handles them separately from this list.
   */
  public List<Element> getGamePieceObstacles() {
    return GAME_PIECE_ELEMENTS;
  }

  /** The blue alliance's HUB collision element (a 47in square, 4 corners). */
  public Element getBlueHub() {
    return BLUE_HUB;
  }

  /** The red alliance's HUB collision element (a 47in square, 4 corners). */
  public Element getRedHub() {
    return RED_HUB;
  }

  /** Center of the blue alliance's HUB, in field-relative meters. */
  public Translation2d getBlueHubCenter() {
    return new Translation2d(BLUE_HUB_CENTER_X, HUB_CENTER_Y);
  }

  /** Center of the red alliance's HUB, in field-relative meters. */
  public Translation2d getRedHubCenter() {
    return new Translation2d(RED_HUB_CENTER_X, HUB_CENTER_Y);
  }

  /**
   * Height of a HUB's open goal above the carpet, in meters. FUEL passing downwards through this
   * height within {@link #getHubEntryRadius()} of the HUB center has been SCORED.
   */
  public double getHubEntryHeight() {
    return HUB_ENTRY_HEIGHT;
  }

  /** Radius of a HUB's open goal about the HUB center, in meters. */
  public double getHubEntryRadius() {
    return HUB_ENTRY_RADIUS;
  }

  /**
   * Where FUEL scored in the blue HUB re-enters play. Sits just outside the HUB's
   * field-center-facing wall, so the direction from {@link #getBlueHubCenter()} to this point is
   * also the direction scored FUEL is dispersed.
   */
  public Translation3d getBlueHubExit() {
    return BLUE_HUB_EXIT;
  }

  /** Where FUEL scored in the red HUB re-enters play. See {@link #getBlueHubExit()}. */
  public Translation3d getRedHubExit() {
    return RED_HUB_EXIT;
  }

  /**
   * The blue HUB's backing net -- a thin, tall element hanging between the HUB and the FIELD
   * center that catches overshot shots. Excluded from {@link #getGamePieceObstacles()}; see there.
   */
  public Element getBlueHubNet() {
    return BLUE_HUB_NET;
  }

  /** The red HUB's backing net. See {@link #getBlueHubNet()}. */
  public Element getRedHubNet() {
    return RED_HUB_NET;
  }

  /**
   * The blue DEPOT, where the blue alliance's FUEL starts the match. Its ~1in barrier is not a
   * meaningful obstacle for either robots or FUEL, so this element is excluded from both
   * {@link #getDriveObstacles()} and {@link #getGamePieceObstacles()}; it exists to place
   * starting FUEL.
   */
  public Element getBlueDepot() {
    return BLUE_DEPOT;
  }

  /** The red DEPOT. See {@link #getBlueDepot()}. */
  public Element getRedDepot() {
    return RED_DEPOT;
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

  /**
   * The blue alliance's TRENCH BAR elements (south, then north) -- the rails on top of each
   * TRENCH. Robots drive under them; game pieces do not.
   */
  public Element[] getBlueTrenchBars() {
    return new Element[] {BLUE_SOUTH_TRENCH_BAR, BLUE_NORTH_TRENCH_BAR};
  }

  /** The red alliance's TRENCH BAR elements (south, then north). See {@link #getBlueTrenchBars()}. */
  public Element[] getRedTrenchBars() {
    return new Element[] {RED_SOUTH_TRENCH_BAR, RED_NORTH_TRENCH_BAR};
  }

  /**
   * The blue BUMP's four sloped ramp faces. Robots drive over the BUMP, so these are absent from
   * {@link #getDriveObstacles()}, but FUEL rolls up and over them.
   */
  public List<Element> getBlueBumpFaces() {
    return BLUE_BUMP_FACES;
  }

  /** The red BUMP's four sloped ramp faces. See {@link #getBlueBumpFaces()}. */
  public List<Element> getRedBumpFaces() {
    return RED_BUMP_FACES;
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
