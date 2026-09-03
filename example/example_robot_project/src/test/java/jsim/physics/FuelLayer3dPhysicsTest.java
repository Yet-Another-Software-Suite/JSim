package jsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import jsim.physics.layers.Cuboid3d;
import jsim.physics.layers.FieldLayout.Element;
import jsim.physics.layers.fields.Field2026;
import jsim.physics.layers.FuelLayer;
import jsim.physics.layers.gamepieces.Fuel2026;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.Meters;

/**
 * Behavioural tests for the 3D FUEL simulation, checking it against the {@link Field2026} geometry
 * it is driven by rather than against hardcoded coordinates.
 */
class FuelLayer3dPhysicsTest {

  private static final Field2026 FIELD = new Field2026();
  private static final double FUEL_RADIUS = FuelLayer.FUEL_DIAMETER.in(Meters) / 2.0;
  private static final Translation2d ROBOT_DIMENSIONS = new Translation2d(0.4, 0.4);

  /** A pose far from any structure or starting FUEL, so the robot never interferes with a test. */
  private static final Pose2d PARKED = new Pose2d(8.27, 0.6, Rotation2d.kZero);

  private static FuelLayer emptyLayer() {
    FuelLayer layer = new FuelLayer(FIELD, "fuel_physics_test").withLoggingFrequency(0);
    layer.clearFuel();
    return layer;
  }

  private static void run(FuelLayer layer, Pose2d pose, ChassisSpeeds speeds, double seconds) {
    for (int i = 0; i < (int) Math.round(seconds / 0.02); i++) {
      layer.process(pose, speeds, ROBOT_DIMENSIONS, 0.02);
    }
  }

  private static void run(FuelLayer layer, double seconds) {
    run(layer, PARKED, new ChassisSpeeds(), seconds);
  }

  @Test
  void startingFuelFillsTheNeutralZoneAndBothDepots() {
    FuelLayer layer = new FuelLayer(FIELD, "fuel_starting_test").withLoggingFrequency(0);

    // 15x6 stacks either side of the center line (360) plus a 3x4 block either side of each
    // DEPOT's center line (48).
    assertEquals(408, layer.getFuelPieces().size());
    for (Fuel2026 fuel : layer.getFuelPieces()) {
      assertEquals(FUEL_RADIUS, fuel.getPosition().getZ(), 1e-9,
          "Starting FUEL should be resting on the carpet");
    }
  }

  @Test
  void droppedFuelSettlesOnTheCarpet() {
    FuelLayer layer = emptyLayer();
    Fuel2026 fuel = layer.spawnFuel(new Translation3d(8.27, 4.0, 1.5));

    run(layer, 4.0);

    assertEquals(FUEL_RADIUS, fuel.getPosition().getZ(), 1e-3,
        "FUEL should come to rest one radius above the carpet");
    assertTrue(fuel.isSupported(), "Resting FUEL should be marked supported");
    assertTrue(Math.abs(fuel.getVelocity().getZ()) < 1e-6, "Resting FUEL should not be bouncing");
  }

  @Test
  void fuelDroppedOnATrenchRestsOnTopOfIt() {
    Element gate = FIELD.getBlueSouthTrenchGate();
    Element bar = FIELD.getBlueTrenchBars()[0];
    // Somewhere on the gate's top face, clear of the TRENCH BAR crossing over it.
    double x = (gate.getMinX() + bar.getMinX()) / 2.0;
    double y = gate.getCenter().getY();

    FuelLayer layer = emptyLayer();
    Fuel2026 fuel = layer.spawnFuel(new Translation3d(x, y, gate.getTopHeight() + 0.6));

    run(layer, 4.0);

    assertEquals(gate.getTopHeight() + FUEL_RADIUS, fuel.getPosition().getZ(), 1e-3,
        "FUEL should rest on the TRENCH GATE's top face, not fall through to the carpet");
  }

  @Test
  void fuelRollsOverABumpInsteadOfStoppingAgainstIt() {
    Element ascent = FIELD.getBlueBumpFaces().get(0);
    Element descent = FIELD.getBlueBumpFaces().get(1);
    double y = ascent.getCenter().getY();

    FuelLayer layer = emptyLayer();
    Fuel2026 fuel = layer.spawnFuel(
        new Translation3d(ascent.getMinX() - 0.5, y, FUEL_RADIUS),
        new Translation3d(4.0, 0, 0));

    run(layer, 1.0);

    assertTrue(fuel.getPosition().getX() > descent.getMaxX(),
        "FUEL driven at the BUMP should climb it and roll off the far side, got x="
            + fuel.getPosition().getX());
    assertEquals(FUEL_RADIUS, fuel.getPosition().getZ(), 0.05,
        "FUEL should be back down near the carpet past the BUMP");
  }

  @Test
  void fuelDroppedIntoTheHubScoresAndIsDispersedBackOntoTheField() {
    Translation2d hubCenter = FIELD.getBlueHubCenter();

    FuelLayer layer = emptyLayer();
    Fuel2026 fuel = layer.spawnFuel(
        new Translation3d(hubCenter.getX(), hubCenter.getY(), FIELD.getHubEntryHeight() + 0.05),
        new Translation3d(0, 0, -3.0));

    run(layer, 0.02);

    assertEquals(1, layer.getBlueHub().getScore(), "Blue HUB should have counted the FUEL");
    assertEquals(0, layer.getRedHub().getScore(), "Red HUB should be untouched");
    assertEquals(FIELD.getBlueHubExit().getX(), fuel.getPosition().getX(), 1e-9,
        "Scored FUEL should re-enter play at the HUB exit");

    run(layer, 2.0);
    assertTrue(fuel.getPosition().getX() > FIELD.getBlueHubExit().getX(),
        "Scored FUEL should be dispersed away from the HUB, towards the FIELD center");
  }

  @Test
  void fuelBouncesOffTheHubWallBelowTheGoal() {
    Element hub = FIELD.getBlueHub();
    double y = FIELD.getBlueHubCenter().getY();

    FuelLayer layer = emptyLayer();
    Fuel2026 fuel = layer.spawnFuel(
        new Translation3d(hub.getMinX() - 0.5, y, FUEL_RADIUS),
        new Translation3d(4.0, 0, 0));

    run(layer, 0.5);

    assertTrue(fuel.getPosition().getX() < hub.getMinX() - FUEL_RADIUS + 1e-6,
        "FUEL should not pass into the HUB's solid base, got x=" + fuel.getPosition().getX());
    assertTrue(fuel.getVelocity().getX() < 0, "FUEL should have bounced back off the HUB wall");
  }

  @Test
  void aDrivingRobotShovesFuelAlongInFrontOfIt() {
    Pose2d robotPose = new Pose2d(8.0, 4.0, Rotation2d.kZero);
    double fuelX = robotPose.getX() + ROBOT_DIMENSIONS.getX() + FUEL_RADIUS - 0.01;

    FuelLayer layer = emptyLayer();
    Fuel2026 fuel = layer.spawnFuel(new Translation3d(fuelX, robotPose.getY(), FUEL_RADIUS));

    run(layer, robotPose, new ChassisSpeeds(1.0, 0, 0), 0.1);

    assertTrue(fuel.getPosition().getX() > fuelX,
        "FUEL should be pushed forwards by the robot driving into it");
    assertTrue(fuel.getVelocity().getX() > 0, "Shoved FUEL should be moving away from the robot");
  }

  @Test
  void fuelAboveTheBumpersPassesOverTheRobot() {
    Pose2d robotPose = new Pose2d(8.0, 4.0, Rotation2d.kZero);
    Distance bumperHeight = Meters.of(0.2);

    FuelLayer layer = emptyLayer().withBumperHeight(bumperHeight);
    Fuel2026 fuel = layer.spawnFuel(
        new Translation3d(robotPose.getX(), robotPose.getY(), bumperHeight.in(Meters) + 0.3),
        new Translation3d(0, 0, 0));

    layer.process(robotPose, new ChassisSpeeds(), ROBOT_DIMENSIONS, 0.02);

    assertEquals(robotPose.getX(), fuel.getPosition().getX(), 1e-9,
        "FUEL flying over the bumpers should not be shoved sideways by the robot");
  }

  @Test
  void anEnabledIntakeRemovesFuelAndFiresItsCallback() {
    Pose2d robotPose = new Pose2d(8.0, 4.0, Rotation2d.kZero);
    boolean[] enabled = {false};
    int[] intakeCount = {0};

    FuelLayer layer = emptyLayer();
    layer.registerIntake(
        new Transform3d(0.3, -0.3, 0.0, Rotation3d.kZero),
        new Transform3d(0.9, 0.3, 0.3, Rotation3d.kZero),
        () -> enabled[0],
        () -> intakeCount[0]++);
    layer.spawnFuel(new Translation3d(
        robotPose.getX() + ROBOT_DIMENSIONS.getX() + FUEL_RADIUS + 0.05,
        robotPose.getY(),
        FUEL_RADIUS));

    run(layer, robotPose, new ChassisSpeeds(), 0.1);
    assertEquals(1, layer.getFuelPieces().size(), "A disabled intake should not pick FUEL up");
    assertEquals(0, intakeCount[0]);

    enabled[0] = true;
    run(layer, robotPose, new ChassisSpeeds(), 0.02);

    assertTrue(layer.getFuelPieces().isEmpty(), "An enabled intake should remove the FUEL");
    assertEquals(1, intakeCount[0], "The intake callback should fire once per FUEL picked up");
  }

  @Test
  void anIntakeBoxFollowsTheRobotAndOnlyPicksUpFuelInsideIt() {
    Transform3d cornerA = new Transform3d(0.3, -0.3, 0.0, Rotation3d.kZero);
    Transform3d cornerB = new Transform3d(0.9, 0.3, 0.3, Rotation3d.kZero);

    FuelLayer layer = emptyLayer();
    layer.registerIntake(cornerA, cornerB);
    // Sat where the intake box will be once the robot has turned 90 degrees to face +Y.
    layer.spawnFuel(new Translation3d(8.0, 4.6, FUEL_RADIUS));

    Pose2d facingPlusX = new Pose2d(8.0, 4.0, Rotation2d.kZero);
    run(layer, facingPlusX, new ChassisSpeeds(), 0.1);
    assertEquals(1, layer.getFuelPieces().size(),
        "FUEL beside the robot should be outside a forward-facing intake box");

    Pose2d facingPlusY = new Pose2d(8.0, 4.0, Rotation2d.kCCW_90deg);
    run(layer, facingPlusY, new ChassisSpeeds(), 0.02);
    assertTrue(layer.getFuelPieces().isEmpty(),
        "Rotating the robot should sweep its intake box onto the FUEL");

    Cuboid3d box = layer.getIntakes().get(0).getBox();
    assertEquals(0.6, box.getXWidth(), 1e-9);
    assertEquals(0.6, box.getYWidth(), 1e-9);
    assertEquals(0.3, box.getZWidth(), 1e-9);
    assertEquals(facingPlusY.getX() + 0.0, box.getCenter().getX(), 1e-9,
        "Facing +Y, the box's forward offset should land on the robot's Y axis");
    assertEquals(facingPlusY.getY() + 0.6, box.getCenter().getY(), 1e-9);
  }

  @Test
  void anIntakeBoxDoesNotReachFuelAboveIt() {
    Pose2d robotPose = new Pose2d(8.0, 4.0, Rotation2d.kZero);

    FuelLayer layer = emptyLayer();
    layer.registerIntake(
        new Transform3d(0.3, -0.3, 0.0, Rotation3d.kZero),
        new Transform3d(0.9, 0.3, 0.2, Rotation3d.kZero));
    // Held stationary well above the top of the pickup box.
    Fuel2026 fuel = layer.spawnFuel(new Translation3d(8.6, 4.0, 1.2));

    layer.process(robotPose, new ChassisSpeeds(), ROBOT_DIMENSIONS, 0.02);

    assertEquals(1, layer.getFuelPieces().size(),
        "FUEL above the pickup box should not be intaked");
    assertTrue(fuel.getPosition().getZ() > 0.2, "The FUEL should still be falling towards the box");
  }

  @Test
  void aFlatFootprintIntakeSpansTheBumperHeight() {
    Pose2d robotPose = new Pose2d(8.0, 4.0, Rotation2d.kZero);

    FuelLayer layer = emptyLayer().withBumperHeight(Meters.of(0.25));
    layer.registerIntake(
        new Transform2d(0.3, -0.3, Rotation2d.kZero),
        new Transform2d(0.9, 0.3, Rotation2d.kZero));

    Cuboid3d box = layer.getIntakes().get(0).getBox();
    assertEquals(0.25, box.getZWidth(), 1e-9,
        "A Transform2d intake should reach from the carpet to the bumper height");

    layer.spawnFuel(new Translation3d(8.6, 4.0, FUEL_RADIUS));
    run(layer, robotPose, new ChassisSpeeds(), 0.02);

    assertTrue(layer.getFuelPieces().isEmpty(), "Ground FUEL should fall inside the flat box");
  }

  @Test
  void overlappingFuelPushesItselfApart() {
    FuelLayer layer = emptyLayer();
    Fuel2026 left = layer.spawnFuel(new Translation3d(8.27, 4.0, FUEL_RADIUS));
    Fuel2026 right = layer.spawnFuel(new Translation3d(8.27 + 0.05, 4.0, FUEL_RADIUS));

    run(layer, 0.1);

    assertTrue(left.getPosition().getDistance(right.getPosition()) >= FUEL_RADIUS * 2.0 - 1e-6,
        "Overlapping FUEL should separate to at least one diameter apart");
  }

  @Test
  void fuelStaysInsideTheFieldPerimeter() {
    FuelLayer layer = emptyLayer();
    Fuel2026 fuel = layer.spawnFuel(
        new Translation3d(FIELD.getFieldLength() / 2.0, FIELD.getFieldWidth() / 2.0, 2.5),
        new Translation3d(25.0, 12.0, 4.0));

    run(layer, 5.0);

    Translation3d position = fuel.getPosition();
    assertTrue(position.getX() >= 0 && position.getX() <= FIELD.getFieldLength(),
        "FUEL should never leave the FIELD along X, got x=" + position.getX());
    assertTrue(position.getY() >= 0 && position.getY() <= FIELD.getFieldWidth(),
        "FUEL should never leave the FIELD along Y, got y=" + position.getY());
    assertFalse(Double.isNaN(position.getZ()), "FUEL position should stay finite");
  }

  @Test
  void aFieldOfFuelSettlesWithoutDriftingOffTheCarpet() {
    FuelLayer layer = new FuelLayer(FIELD, "fuel_soak_test").withLoggingFrequency(0);

    run(layer, 1.0);

    for (Fuel2026 fuel : layer.getFuelPieces()) {
      Translation3d position = fuel.getPosition();
      assertTrue(position.getZ() >= FUEL_RADIUS - 1e-6,
          "No FUEL should sink through the carpet, got z=" + position.getZ());
      assertTrue(position.getX() >= 0 && position.getX() <= FIELD.getFieldLength(),
          "No FUEL should be squeezed off the FIELD, got x=" + position.getX());
      assertTrue(position.getY() >= 0 && position.getY() <= FIELD.getFieldWidth(),
          "No FUEL should be squeezed off the FIELD, got y=" + position.getY());
    }
  }
}
