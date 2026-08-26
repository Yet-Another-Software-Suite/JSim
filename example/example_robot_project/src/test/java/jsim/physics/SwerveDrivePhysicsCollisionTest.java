package jsim.physics;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static jsim.physics.layers.fields.Field2026.FIELD_LENGTH;
import static jsim.physics.layers.fields.Field2026.FIELD_WIDTH;
import static jsim.physics.layers.fields.Field2026.HUB_DISTANCE_FROM_WALL;
import static jsim.physics.layers.fields.Field2026.HUB_SIZE;
import static jsim.physics.layers.fields.Field2026.TOWER_DEPTH;
import static jsim.physics.layers.fields.Field2026.WALL_THICKNESS;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import jsim.physics.layers.Dyn4jCollisionLayer;
import jsim.physics.layers.fields.Field2026;
import org.junit.jupiter.api.Test;

/**
 * Integration tests driving a simulated robot through the {@link Dyn4jCollisionLayer} against the
 * real {@link Field2026} layout, verifying that every boundary wall and every field element brings
 * the robot to a stop instead of letting it tunnel through.
 *
 * <p>Expected collision points below are derived directly from {@link Field2026}'s own public
 * geometry constants, so they stay in sync with the actual field definition under test.
 */
class SwerveDrivePhysicsCollisionTest {

  private static final double CENTER_Y = FIELD_WIDTH / 2.0;

  /** Row X of the blue (near, wallX=0) alliance HUB/TOWER structures. */
  private static final double BLUE_HUB_X = HUB_DISTANCE_FROM_WALL;
  private static final double BLUE_TOWER_X = TOWER_DEPTH / 2.0;

  /** Row X of the red (far, wallX=FIELD_LENGTH) alliance HUB/TOWER structures. */
  private static final double RED_HUB_X = FIELD_LENGTH - HUB_DISTANCE_FROM_WALL;
  private static final double RED_TOWER_X = FIELD_LENGTH - TOWER_DEPTH / 2.0;

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

  private static SwerveDrivePhysics newPhysics(Pose2d initialPose) {
    Translation2d[] moduleLocations = {
        new Translation2d(0.3, 0.3),
        new Translation2d(0.3, -0.3),
        new Translation2d(-0.3, 0.3),
        new Translation2d(-0.3, -0.3)
    };
    SwerveDrivePhysics physics = new SwerveDrivePhysics(
        moduleLocations,
        Meters.of(ROBOT_SIZE),
        Meters.of(ROBOT_SIZE),
        initialPose,
        zeroModulePositions());
    return physics.addLayer(new Dyn4jCollisionLayer(Kilograms.of(50.0), new Field2026()));
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

    double expectedStopX = WALL_THICKNESS / 2.0 + ROBOT_HALF;
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

    double expectedStopX = FIELD_LENGTH - WALL_THICKNESS / 2.0 - ROBOT_HALF;
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

    double expectedStopY = WALL_THICKNESS / 2.0 + ROBOT_HALF;
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

    double expectedStopY = FIELD_WIDTH - WALL_THICKNESS / 2.0 - ROBOT_HALF;
    assertTrue(finalPose.getY() <= expectedStopY + POSITION_TOLERANCE_METERS,
        "Robot tunneled through the north guardrail: y=" + finalPose.getY());
    assertTrue(finalPose.getY() >= expectedStopY - POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the north guardrail: y=" + finalPose.getY());
  }

  @Test
  void stopsAtBlueHub() {
    Pose2d start = new Pose2d(BLUE_HUB_X + 2.0, CENTER_Y, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(-APPROACH_SPEED, 0, 0));

    double expectedStopX = BLUE_HUB_X + HUB_SIZE / 2.0 + ROBOT_HALF;
    assertTrue(finalPose.getX() >= expectedStopX - POSITION_TOLERANCE_METERS,
        "Robot tunneled through the blue HUB: x=" + finalPose.getX());
    assertTrue(finalPose.getX() <= expectedStopX + POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the blue HUB: x=" + finalPose.getX());
  }

  @Test
  void stopsAtRedHub() {
    Pose2d start = new Pose2d(RED_HUB_X - 2.0, CENTER_Y, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(APPROACH_SPEED, 0, 0));

    double expectedStopX = RED_HUB_X - HUB_SIZE / 2.0 - ROBOT_HALF;
    assertTrue(finalPose.getX() <= expectedStopX + POSITION_TOLERANCE_METERS,
        "Robot tunneled through the red HUB: x=" + finalPose.getX());
    assertTrue(finalPose.getX() >= expectedStopX - POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the red HUB: x=" + finalPose.getX());
  }

  @Test
  void stopsAtBlueTower() {
    double startX = (BLUE_TOWER_X + BLUE_HUB_X) / 2.0;
    Pose2d start = new Pose2d(startX, CENTER_Y, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(-APPROACH_SPEED, 0, 0));

    double expectedStopX = TOWER_DEPTH + ROBOT_HALF;
    assertTrue(finalPose.getX() >= expectedStopX - POSITION_TOLERANCE_METERS,
        "Robot tunneled through the blue TOWER: x=" + finalPose.getX());
    assertTrue(finalPose.getX() <= expectedStopX + POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the blue TOWER: x=" + finalPose.getX());
  }

  @Test
  void stopsAtRedTower() {
    double startX = (RED_TOWER_X + RED_HUB_X) / 2.0;
    Pose2d start = new Pose2d(startX, CENTER_Y, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d finalPose = driveUntilSettled(physics, new ChassisSpeeds(APPROACH_SPEED, 0, 0));

    double expectedStopX = FIELD_LENGTH - TOWER_DEPTH - ROBOT_HALF;
    assertTrue(finalPose.getX() <= expectedStopX + POSITION_TOLERANCE_METERS,
        "Robot tunneled through the red TOWER: x=" + finalPose.getX());
    assertTrue(finalPose.getX() >= expectedStopX - POSITION_TOLERANCE_METERS,
        "Robot stopped too far from the red TOWER: x=" + finalPose.getX());
  }
}
