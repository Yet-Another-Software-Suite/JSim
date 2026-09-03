package jsim.physics;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
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
 * Integration tests for {@link SwerveDrivePhysics#resetOdometry}, verifying it re-baselines the
 * ground-truth pose, physical velocity, and odometry estimate, and that the reset actually reaches
 * into a registered {@link Dyn4jCollisionLayer}'s internal rigid body rather than only updating
 * {@code SwerveDrivePhysics}'s own bookkeeping.
 */
class SwerveDrivePhysicsResetOdometryTest {

  private static final Field2026 FIELD = new Field2026();
  private static final double FIELD_LENGTH = FIELD.getFieldLength();
  private static final double FIELD_WIDTH = FIELD.getFieldWidth();

  private static final double ROBOT_SIZE = 0.9;
  private static final double ROBOT_HALF = ROBOT_SIZE / 2.0;
  private static final double APPROACH_SPEED = 4.0; // m/s
  private static final double DT = Dyn4jCollisionLayer.DEFAULT_DT_SECONDS;

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

  private static Pose2d step(SwerveDrivePhysics physics, ChassisSpeeds commanded, int steps) {
    SwerveModulePosition[] positions = zeroModulePositions();
    Pose2d pose = null;
    for (int i = 0; i < steps; i++) {
      pose = physics.update(commanded, Rotation2d.kZero, positions, DT).pose();
    }
    return pose;
  }

  @Test
  void resetToStationaryClearsGroundTruthAndOdometryImmediately() {
    Pose2d start = new Pose2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    // Get the robot moving so there is real velocity/pose state to discard.
    step(physics, new ChassisSpeeds(APPROACH_SPEED, 0, 0), 25);
    assertTrue(physics.getPhysicalSpeeds().vxMetersPerSecond > 1.0,
        "Robot should be moving before the reset for this test to be meaningful");

    Pose2d resetPose = new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(90));
    physics.resetOdometry(resetPose);

    assertEquals(resetPose.getX(), physics.getPose().getX(), 1e-9);
    assertEquals(resetPose.getY(), physics.getPose().getY(), 1e-9);
    assertEquals(resetPose.getRotation().getRadians(), physics.getPose().getRotation().getRadians(), 1e-9);
    assertEquals(0.0, physics.getPhysicalSpeeds().vxMetersPerSecond, 1e-9);
    assertEquals(0.0, physics.getPhysicalSpeeds().vyMetersPerSecond, 1e-9);
    assertEquals(0.0, physics.getPhysicalSpeeds().omegaRadiansPerSecond, 1e-9);

    Pose2d odometryPose = physics.getOdometry().getEstimatedPosition();
    assertEquals(resetPose.getX(), odometryPose.getX(), 1e-9);
    assertEquals(resetPose.getY(), odometryPose.getY(), 1e-9);

    // With no commanded speed, a single step after reset should barely move the robot at all --
    // proving the reset actually discarded the pre-reset momentum rather than letting it carry
    // through into the next update() call.
    Pose2d afterOneIdleStep = step(physics, new ChassisSpeeds(), 1);
    double drift = resetPose.getTranslation().getDistance(afterOneIdleStep.getTranslation());
    assertTrue(drift < 0.01,
        "Robot drifted " + drift + "m on the first idle step after reset; expected leftover "
            + "pre-reset momentum to have been discarded");
  }

  @Test
  void resetWithSpeedsResumesMotionAtThatVelocity() {
    Pose2d start = new Pose2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    Pose2d resetPose = new Pose2d(5.0, 5.0, Rotation2d.kZero);
    ChassisSpeeds resetSpeeds = new ChassisSpeeds(APPROACH_SPEED, 0, 0);
    physics.resetOdometry(resetPose, resetSpeeds);

    assertEquals(APPROACH_SPEED, physics.getPhysicalSpeeds().vxMetersPerSecond, 1e-9,
        "getPhysicalSpeeds() should reflect the reset velocity immediately, before any update() call");

    // Commanding the same speed the robot was reset to should continue smoothly in free space,
    // covering roughly speed * dt * steps with no artificial jump or stall on the first step.
    int steps = 25;
    Pose2d finalPose = step(physics, resetSpeeds, steps);
    double expectedX = resetPose.getX() + APPROACH_SPEED * DT * steps;
    assertEquals(expectedX, finalPose.getX(), 0.05,
        "Robot did not continue at the reset velocity as expected: x=" + finalPose.getX());
  }

  @Test
  void resetRelocatesCollisionLayerBodySoOldWallContactDoesNotLinger() {
    // Drive the robot into the west alliance wall and let it settle there, so the
    // Dyn4jCollisionLayer's internal rigid body is sitting right at x~0 in active contact.
    Pose2d wallStart = new Pose2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0 + 2.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(wallStart);
    step(physics, new ChassisSpeeds(-APPROACH_SPEED, 0, 0), 150);
    assertTrue(physics.getPose().getX() < 1.0,
        "Robot should have settled against the west wall before the reset for this test to be meaningful");

    // Teleport far from any field structure and resume driving in free space. Field center is
    // roughly equidistant from the blue and red HUB/TOWER rows, so a short hop in either +/-X
    // direction has plenty of clearance from both.
    Pose2d clearPose = new Pose2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0, Rotation2d.kZero);
    physics.resetOdometry(clearPose);

    int steps = 25;
    ChassisSpeeds freeSpaceSpeeds = new ChassisSpeeds(APPROACH_SPEED, 0, 0);
    Pose2d finalPose = step(physics, freeSpaceSpeeds, steps);

    double expectedX = clearPose.getX() + APPROACH_SPEED * DT * steps;
    assertEquals(expectedX, finalPose.getX(), 0.05,
        "Robot did not move freely after being reset away from the wall -- the collision layer's "
            + "internal body may still be referencing its pre-reset position/contact: x=" + finalPose.getX());
  }

  @Test
  void resetNearAWallStillCollidesCorrectlyAtTheNewPosition() {
    Pose2d farAway = new Pose2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(farAway);
    // Establish the Dyn4jCollisionLayer's body/world (lazily initialized on first process() call).
    step(physics, new ChassisSpeeds(), 1);

    // Reset to a stationary position with just enough clearance from the south guardrail (y=0)
    // to approach it fresh.
    Pose2d nearSouthWall = new Pose2d(FIELD_LENGTH / 2.0, 2.0, Rotation2d.kZero);
    physics.resetOdometry(nearSouthWall);

    Pose2d finalPose = step(physics, new ChassisSpeeds(0, -APPROACH_SPEED, 0), 150);

    double expectedStopY = bounds(FIELD.getSouthGuardrail())[3] + ROBOT_HALF;
    assertTrue(finalPose.getY() >= expectedStopY - 0.25,
        "Robot tunneled through the south guardrail after reset: y=" + finalPose.getY());
    assertTrue(finalPose.getY() <= expectedStopY + 0.25,
        "Robot stopped too far from the south guardrail after reset: y=" + finalPose.getY());
  }
}
