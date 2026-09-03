package jsim.physics;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import jsim.physics.layers.Dyn4jCollisionLayer;
import jsim.physics.layers.fields.FieldLayout;
import jsim.physics.layers.fields.Field2026;

import org.junit.jupiter.api.Test;

/**
 * Integration tests driving a simulated robot through the {@link Dyn4jCollisionLayer} against the
 * real {@link Field2026} layout, verifying that every boundary wall and every field element brings
 * the robot to a stop instead of letting it tunnel through.
 *
 * <p>Expected collision points below are derived directly from the actual {@link Field2026}
 * instance's collision elements (via their real vertex bounds), rather than from separate
 * hand-picked constants, so they stay in sync with whichever AprilTag field variant is loaded.
 */
class SwerveDrivePhysicsCollisionTest {

  private static final Field2026 FIELD = new Field2026();
  private static final double FIELD_LENGTH = FIELD.getFieldLength();
  private static final double FIELD_WIDTH = FIELD.getFieldWidth();
  private static final double CENTER_Y = FIELD_WIDTH / 2.0;

  /** Bumper-to-bumper square footprint used for the simulated test robot. */
  private static final double ROBOT_SIZE = 0.9;
  private static final double ROBOT_HALF = ROBOT_SIZE / 2.0;

  private static final double APPROACH_SPEED = 4.0; // m/s
  private static final int TOTAL_STEPS = 300; // 6s of sim time at DEFAULT_DT_SECONDS
  // The collision layer re-commands the full approach speed into the obstacle every frame, so
  // rather than coming to a perfectly stationary halt, contact settles into a small bounded
  // sawtooth oscillation (confirmed stable over an 80s soak, never widening or tunneling through).
  // A short trailing window still clearly distinguishes "held at the obstacle" (oscillation of a
  // few cm) from "drove straight through" (multiple meters), without being tripped up by it.
  private static final int SETTLE_WINDOW_STEPS = 50; // 1s
  private static final double SETTLE_TOLERANCE_METERS = 0.15;
  private static final double POSITION_TOLERANCE_METERS = 0.25;

  /** Returns {minX, maxX, minY, maxY} over an element's vertices, in field-relative meters. */
  private static double[] bounds(FieldLayout.Element element) {
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

  /**
   * Returns the midpoint Y of the flat face at the given extreme X (the vertices exactly at that
   * X). Needed for non-rectangular elements like the octagonal HUB: its bounding-box center Y sits
   * exactly on the boundary between its two side faces (each spans only half the total height), so
   * approaching at that center grazes a single vertex instead of crossing a solid face.
   */
  private static double faceMidpointY(FieldLayout.Element element, double extremeX) {
    double sumY = 0;
    int count = 0;
    for (Translation2d v : element.getVertices()) {
      if (Math.abs(v.getX() - extremeX) < 1e-6) {
        sumY += v.getY();
        count++;
      }
    }
    return sumY / count;
  }

  private static SwerveDrivePhysics newPhysics(Pose2d initialPose) {
    return newPhysics(initialPose, ROBOT_SIZE);
  }

  private static SwerveDrivePhysics newPhysics(Pose2d initialPose, double robotSize) {
    Translation2d[] moduleLocations = {
        new Translation2d(0.3, 0.3),
        new Translation2d(0.3, -0.3),
        new Translation2d(-0.3, 0.3),
        new Translation2d(-0.3, -0.3)
    };
    SwerveDrivePhysics physics = new SwerveDrivePhysics(
        moduleLocations,
        Meters.of(robotSize),
        Meters.of(robotSize),
        initialPose,
        zeroModulePositions());
    return physics.addLayer(new Dyn4jCollisionLayer(Kilograms.of(50.0), FIELD));
  }

  private static SwerveModulePosition[] zeroModulePositions() {
    return new SwerveModulePosition[] {
        new SwerveModulePosition(0, new Rotation2d()),
        new SwerveModulePosition(0, new Rotation2d()),
        new SwerveModulePosition(0, new Rotation2d()),
        new SwerveModulePosition(0, new Rotation2d())
    };
  }

  /**
   * Drives {@code physics} with a constant commanded velocity for {@link #TOTAL_STEPS} loop
   * iterations and asserts that the robot's ground-truth pose stops advancing well before the run
   * ends, i.e. that it was actually arrested by a collision rather than driving forever.
   *
   * @return The ground-truth pose after the final step.
   */
  private static Pose2d driveUntilSettled(SwerveDrivePhysics physics, ChassisSpeeds commanded) {
    SwerveModulePosition[] positions = zeroModulePositions();
    Pose2d windowStartPose = null;
    Pose2d pose = null;
    for (int i = 0; i < TOTAL_STEPS; i++) {
      SwerveDrivePhysics.PhysicsState state =
          physics.update(commanded, Rotation2d.kZero, positions, Dyn4jCollisionLayer.DEFAULT_DT_SECONDS);
      pose = state.pose();
      if (i == TOTAL_STEPS - SETTLE_WINDOW_STEPS) {
        windowStartPose = pose;
      }
    }
    double driftInFinalWindow = windowStartPose.getTranslation().getDistance(pose.getTranslation());
    assertTrue(
        driftInFinalWindow < SETTLE_TOLERANCE_METERS,
        "Robot moved " + driftInFinalWindow + "m in the final " + SETTLE_WINDOW_STEPS + " steps; "
            + "expected it to be held in place by a collision well before the run ended");
    return pose;
  }

  @Test
  void stopsAtWestAllianceWall() {
    Pose2d start = new Pose2d(FIELD_LENGTH / 2.0, CENTER_Y + 2.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(-APPROACH_SPEED, 0, 0));

    double expectedStopX = bounds(FIELD.getWestAllianceWall())[1] + ROBOT_HALF;
    assertTrue(finalPose.getX() >= expectedStopX - POSITION_TOLERANCE_METERS,
        "Robot tunneled through the west alliance wall: x=" + finalPose.getX());
    assertTrue(finalPose.getX() <= expectedStopX + POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the west alliance wall: x=" + finalPose.getX());
  }

  @Test
  void stopsAtEastAllianceWall() {
    Pose2d start = new Pose2d(FIELD_LENGTH / 2.0, CENTER_Y + 2.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(APPROACH_SPEED, 0, 0));

    double expectedStopX = bounds(FIELD.getEastAllianceWall())[0] - ROBOT_HALF;
    assertTrue(finalPose.getX() <= expectedStopX + POSITION_TOLERANCE_METERS,
        "Robot tunneled through the east alliance wall: x=" + finalPose.getX());
    assertTrue(finalPose.getX() >= expectedStopX - POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the east alliance wall: x=" + finalPose.getX());
  }

  @Test
  void stopsAtSouthGuardrail() {
    Pose2d start = new Pose2d(FIELD_LENGTH / 2.0, CENTER_Y, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(0, -APPROACH_SPEED, 0));

    double expectedStopY = bounds(FIELD.getSouthGuardrail())[3] + ROBOT_HALF;
    assertTrue(finalPose.getY() >= expectedStopY - POSITION_TOLERANCE_METERS,
        "Robot tunneled through the south guardrail: y=" + finalPose.getY());
    assertTrue(finalPose.getY() <= expectedStopY + POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the south guardrail: y=" + finalPose.getY());
  }

  @Test
  void stopsAtNorthGuardrail() {
    Pose2d start = new Pose2d(FIELD_LENGTH / 2.0, CENTER_Y, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(0, APPROACH_SPEED, 0));

    double expectedStopY = bounds(FIELD.getNorthGuardrail())[2] - ROBOT_HALF;
    assertTrue(finalPose.getY() <= expectedStopY + POSITION_TOLERANCE_METERS,
        "Robot tunneled through the north guardrail: y=" + finalPose.getY());
    assertTrue(finalPose.getY() >= expectedStopY - POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the north guardrail: y=" + finalPose.getY());
  }

  @Test
  void stopsAtBlueHub() {
    double[] hubBounds = bounds(FIELD.getBlueHub());
    double approachY = faceMidpointY(FIELD.getBlueHub(), hubBounds[1]);
    Pose2d start = new Pose2d(hubBounds[1] + 2.0, approachY, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(-APPROACH_SPEED, 0, 0));

    double expectedStopX = hubBounds[1] + ROBOT_HALF;
    assertTrue(finalPose.getX() >= expectedStopX - POSITION_TOLERANCE_METERS,
        "Robot tunneled through the blue HUB: x=" + finalPose.getX());
    assertTrue(finalPose.getX() <= expectedStopX + POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the blue HUB: x=" + finalPose.getX());
  }

  @Test
  void stopsAtRedHub() {
    double[] hubBounds = bounds(FIELD.getRedHub());
    double approachY = faceMidpointY(FIELD.getRedHub(), hubBounds[0]);
    Pose2d start = new Pose2d(hubBounds[0] - 2.0, approachY, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(APPROACH_SPEED, 0, 0));

    double expectedStopX = hubBounds[0] - ROBOT_HALF;
    assertTrue(finalPose.getX() <= expectedStopX + POSITION_TOLERANCE_METERS,
        "Robot tunneled through the red HUB: x=" + finalPose.getX());
    assertTrue(finalPose.getX() >= expectedStopX - POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the red HUB: x=" + finalPose.getX());
  }

  @Test
  void stopsAtBlueTowerUpright() {
    double[] uprightBounds = bounds(FIELD.getBlueTowerUprights()[0]);
    double[] hubBounds = bounds(FIELD.getBlueHub());
    double uprightCenterX = (uprightBounds[0] + uprightBounds[1]) / 2.0;
    double uprightCenterY = (uprightBounds[2] + uprightBounds[3]) / 2.0;
    double hubCenterX = (hubBounds[0] + hubBounds[1]) / 2.0;
    double startX = (uprightCenterX + hubCenterX) / 2.0;
    Pose2d start = new Pose2d(startX, uprightCenterY, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(-APPROACH_SPEED, 0, 0));

    double expectedStopX = uprightBounds[1] + ROBOT_HALF;
    assertTrue(finalPose.getX() >= expectedStopX - POSITION_TOLERANCE_METERS,
        "Robot tunneled through the blue TOWER's upright: x=" + finalPose.getX());
    assertTrue(finalPose.getX() <= expectedStopX + POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the blue TOWER's upright: x=" + finalPose.getX());
  }

  @Test
  void stopsAtRedTowerUpright() {
    double[] uprightBounds = bounds(FIELD.getRedTowerUprights()[0]);
    double[] hubBounds = bounds(FIELD.getRedHub());
    double uprightCenterX = (uprightBounds[0] + uprightBounds[1]) / 2.0;
    double uprightCenterY = (uprightBounds[2] + uprightBounds[3]) / 2.0;
    double hubCenterX = (hubBounds[0] + hubBounds[1]) / 2.0;
    double startX = (uprightCenterX + hubCenterX) / 2.0;
    Pose2d start = new Pose2d(startX, uprightCenterY, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(APPROACH_SPEED, 0, 0));

    double expectedStopX = uprightBounds[0] - ROBOT_HALF;
    assertTrue(finalPose.getX() <= expectedStopX + POSITION_TOLERANCE_METERS,
        "Robot tunneled through the red TOWER's upright: x=" + finalPose.getX());
    assertTrue(finalPose.getX() >= expectedStopX - POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the red TOWER's upright: x=" + finalPose.getX());
  }

  @Test
  void robotCanPassBetweenTowerUprights() {
    // Robots can drive under/between a TOWER's uprights -- only the poles themselves collide. Use
    // a robot narrower than the upright gap (32.25in ~ 0.82m) to actually fit through it.
    double narrowRobotSize = 0.6;
    double narrowRobotHalf = narrowRobotSize / 2.0;

    FieldLayout.Element[] uprights = FIELD.getBlueTowerUprights();
    double gapCenterY = ((bounds(uprights[0])[2] + bounds(uprights[0])[3]) / 2.0
        + (bounds(uprights[1])[2] + bounds(uprights[1])[3]) / 2.0) / 2.0;
    double[] hubBounds = bounds(FIELD.getBlueHub());
    double hubCenterX = (hubBounds[0] + hubBounds[1]) / 2.0;
    double uprightCenterX = (bounds(uprights[0])[0] + bounds(uprights[0])[1]) / 2.0;
    double startX = (uprightCenterX + hubCenterX) / 2.0;

    Pose2d start = new Pose2d(startX, gapCenterY, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start, narrowRobotSize);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(-APPROACH_SPEED, 0, 0));

    // Should sail through the gap to the west alliance wall, not get stopped by the TOWER at all.
    double expectedStopX = bounds(FIELD.getWestAllianceWall())[1] + narrowRobotHalf;
    assertTrue(Math.abs(finalPose.getX() - expectedStopX) <= POSITION_TOLERANCE_METERS,
        "Robot should have driven freely between the TOWER's uprights to the west wall: x="
            + finalPose.getX());
  }

  @Test
  void stopsAtBlueSouthTrenchGate() {
    // Approach from directly between the gate and the HUB (open trench-corridor space on both
    // sides), moving south/-Y toward the gate's far face. The gate sits too close to both the
    // guardrail and the HUB for a full 2m clearance offset from either side to stay on the field
    // / clear of the HUB, so use a smaller 1m offset within that open gap instead.
    double[] gateBounds = bounds(FIELD.getBlueSouthTrenchGate());
    double gateCenterX = (gateBounds[0] + gateBounds[1]) / 2.0;
    Pose2d start = new Pose2d(gateCenterX, gateBounds[3] + 1.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(0, -APPROACH_SPEED, 0));

    double expectedStopY = gateBounds[3] + ROBOT_HALF;
    assertTrue(finalPose.getY() >= expectedStopY - POSITION_TOLERANCE_METERS,
        "Robot tunneled through the blue south TRENCH GATE: y=" + finalPose.getY());
    assertTrue(finalPose.getY() <= expectedStopY + POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the blue south TRENCH GATE: y=" + finalPose.getY());
  }
}
