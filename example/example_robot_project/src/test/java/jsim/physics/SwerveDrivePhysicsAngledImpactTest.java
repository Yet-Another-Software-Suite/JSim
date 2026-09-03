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
 * Regression tests for a bug where hitting a field element at an angle (rather than squarely on a
 * face) caused {@link SwerveDrivePhysics}'s ground-truth heading to spin up out of control.
 *
 * <p>Root cause: an off-center (e.g. corner) contact imparts real angular velocity within a single
 * dyn4j step. That got integrated into the ground-truth pose's heading every frame, and since the
 * commanded {@link ChassisSpeeds} are robot-relative, the drifting heading kept reinterpreting the
 * same command along a new field-relative direction each frame -- compounding into a runaway spin
 * that made the robot appear to slide/orbit around the obstacle indefinitely instead of stopping or
 * sliding cleanly along it, even with zero commanded angular velocity.
 *
 * <p>Fix: {@link SwerveDrivePhysics#update} now takes heading directly from the supplied gyro angle
 * instead of integrating the collision layer's angular response, since a real swerve chassis's
 * heading is governed by its own drivetrain/gyro, not by passive collision torque. These tests
 * drive full-speed into a wall, the TOWER, and the HUB at several angles (including squarely at a
 * corner, which is what originally triggered this) and assert the heading never drifts away from
 * the commanded (constant, zero-rotation) gyro angle, and that the robot never ends up on the wrong
 * side of what it hit.
 *
 * <p>Expected geometry is read directly off the real {@link Field2026} instance's collision
 * elements (via their vertex bounds) rather than separate hand-picked constants.
 */
class SwerveDrivePhysicsAngledImpactTest {

  private static final Field2026 FIELD = new Field2026();
  private static final double FIELD_LENGTH = FIELD.getFieldLength();
  private static final double FIELD_WIDTH = FIELD.getFieldWidth();
  private static final double CENTER_Y = FIELD_WIDTH / 2.0;

  private static final double ROBOT_SIZE = 0.9;
  private static final double ROBOT_HALF = ROBOT_SIZE / 2.0;
  private static final double APPROACH_SPEED = 4.0; // m/s
  private static final int STEPS = 300; // 6s of sim time at DEFAULT_DT_SECONDS
  private static final double HEADING_DRIFT_TOLERANCE_DEGREES = 0.01;

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
        new Translation2d(0.3, 0.3), new Translation2d(0.3, -0.3),
        new Translation2d(-0.3, 0.3), new Translation2d(-0.3, -0.3)
    };
    SwerveDrivePhysics physics = new SwerveDrivePhysics(
        moduleLocations, Meters.of(ROBOT_SIZE), Meters.of(ROBOT_SIZE), initialPose, zeroModulePositions());
    return physics.addLayer(new Dyn4jCollisionLayer(Kilograms.of(50.0), FIELD));
  }

  private static SwerveModulePosition[] zeroModulePositions() {
    return new SwerveModulePosition[] {
        new SwerveModulePosition(0, new Rotation2d()), new SwerveModulePosition(0, new Rotation2d()),
        new SwerveModulePosition(0, new Rotation2d()), new SwerveModulePosition(0, new Rotation2d())
    };
  }

  /** Axis-aligned solid-obstacle bounds a robot's center must never enter (that would mean tunneling). */
  private record SolidBounds(double minX, double maxX, double minY, double maxY) {
    static SolidBounds of(double[] b) {
      return new SolidBounds(b[0], b[1], b[2], b[3]);
    }

    boolean contains(Pose2d pose) {
      return pose.getX() > minX && pose.getX() < maxX && pose.getY() > minY && pose.getY() < maxY;
    }
  }

  private static Pose2d driveAndAssertNoHeadingDrift(SwerveDrivePhysics physics, ChassisSpeeds commanded) {
    return driveAndAssertNoHeadingDrift(physics, commanded, null);
  }

  /**
   * Drives {@code physics} at {@code commanded} (with a constant zero-rotation gyro angle) for
   * {@link #STEPS} loop iterations, asserting the ground-truth heading never drifts off of that
   * gyro angle, the pose stays finite and within a generous bound around the field, and (if given)
   * the robot's center never enters {@code solidObstacle} -- sliding far away along a wall after a
   * glancing hit is expected and fine, but the center passing through the obstacle's own solid
   * footprint at any point during the run is unambiguous tunneling.
   */
  private static Pose2d driveAndAssertNoHeadingDrift(
      SwerveDrivePhysics physics, ChassisSpeeds commanded, SolidBounds solidObstacle) {
    SwerveModulePosition[] positions = zeroModulePositions();
    Pose2d pose = null;
    double maxHeadingDriftDegrees = 0.0;
    for (int i = 0; i < STEPS; i++) {
      SwerveDrivePhysics.PhysicsState state =
          physics.update(commanded, Rotation2d.kZero, positions, Dyn4jCollisionLayer.DEFAULT_DT_SECONDS);
      pose = state.pose();
      maxHeadingDriftDegrees = Math.max(maxHeadingDriftDegrees, Math.abs(pose.getRotation().getDegrees()));

      assertTrue(Double.isFinite(pose.getX()) && Double.isFinite(pose.getY()),
          "Pose became non-finite at step " + i + ": " + pose);
      assertTrue(pose.getX() > -1.0 && pose.getX() < FIELD_LENGTH + 1.0
              && pose.getY() > -1.0 && pose.getY() < FIELD_WIDTH + 1.0,
          "Robot left the field bounds at step " + i + ", suggesting a physics blow-up: " + pose);
      if (solidObstacle != null) {
        assertTrue(!solidObstacle.contains(pose),
            "Robot's center passed through solid obstacle geometry at step " + i + ": " + pose);
      }
    }
    assertTrue(maxHeadingDriftDegrees < HEADING_DRIFT_TOLERANCE_DEGREES,
        "Ground-truth heading drifted " + maxHeadingDriftDegrees + " degrees away from the commanded "
            + "zero-rotation gyro angle despite zero commanded angular velocity -- collision torque is "
            + "leaking into the ground-truth heading again.");
    return pose;
  }

  @Test
  void westWallAt30DegreesDoesNotSpinOutOrTunnel() {
    Pose2d start = new Pose2d(3.0, CENTER_Y + 2.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);
    double angle = Math.toRadians(30);
    ChassisSpeeds commanded = new ChassisSpeeds(-APPROACH_SPEED * Math.cos(angle), APPROACH_SPEED * Math.sin(angle), 0);

    Pose2d finalPose = driveAndAssertNoHeadingDrift(physics, commanded);
    double expectedStopX = bounds(FIELD.getWestAllianceWall())[1] + ROBOT_HALF;
    assertTrue(finalPose.getX() >= expectedStopX - 0.25,
        "Robot tunneled through the west alliance wall: " + finalPose);
  }

  @Test
  void westWallAt45DegreesDoesNotSpinOutOrTunnel() {
    Pose2d start = new Pose2d(3.0, CENTER_Y + 2.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);
    double diag = APPROACH_SPEED / Math.sqrt(2);
    ChassisSpeeds commanded = new ChassisSpeeds(-diag, diag, 0);

    Pose2d finalPose = driveAndAssertNoHeadingDrift(physics, commanded);
    double expectedStopX = bounds(FIELD.getWestAllianceWall())[1] + ROBOT_HALF;
    assertTrue(finalPose.getX() >= expectedStopX - 0.25,
        "Robot tunneled through the west alliance wall: " + finalPose);
  }

  @Test
  void westWallAt60DegreesDoesNotSpinOutOrTunnel() {
    Pose2d start = new Pose2d(3.0, CENTER_Y + 2.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);
    double angle = Math.toRadians(60);
    ChassisSpeeds commanded = new ChassisSpeeds(-APPROACH_SPEED * Math.cos(angle), APPROACH_SPEED * Math.sin(angle), 0);

    Pose2d finalPose = driveAndAssertNoHeadingDrift(physics, commanded);
    double expectedStopX = bounds(FIELD.getWestAllianceWall())[1] + ROBOT_HALF;
    assertTrue(finalPose.getX() >= expectedStopX - 0.25,
        "Robot tunneled through the west alliance wall: " + finalPose);
  }

  @Test
  void northGuardrailAt45DegreesDoesNotSpinOutOrTunnel() {
    Pose2d start = new Pose2d(FIELD_LENGTH / 2.0, FIELD_WIDTH - 3.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);
    double diag = APPROACH_SPEED / Math.sqrt(2);
    ChassisSpeeds commanded = new ChassisSpeeds(diag, diag, 0);

    Pose2d finalPose = driveAndAssertNoHeadingDrift(physics, commanded);
    double expectedStopY = bounds(FIELD.getNorthGuardrail())[2] - ROBOT_HALF;
    assertTrue(finalPose.getY() <= expectedStopY + 0.25,
        "Robot tunneled through the north guardrail: " + finalPose);
  }

  @Test
  void blueTowerUprightCornerAt45DegreesDoesNotSpinOutOrTunnel() {
    // This exact corner-on-45-degree hit (against one of the TOWER's UPRIGHT poles) is what
    // originally triggered the runaway heading spin. The upright sits right against the alliance
    // wall, so approach from its far (field-facing) corner to keep the start pose on the field.
    double[] uprightBounds = bounds(FIELD.getBlueTowerUprights()[0]);
    Pose2d start = new Pose2d(uprightBounds[1] + 2.0, uprightBounds[3] + 2.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);
    double diag = APPROACH_SPEED / Math.sqrt(2);
    ChassisSpeeds commanded = new ChassisSpeeds(-diag, -diag, 0);

    // A glancing hit legitimately sliding far away along a wall afterward is fine (and expected);
    // the robot's center must simply never pass through the upright's own solid footprint.
    driveAndAssertNoHeadingDrift(physics, commanded, SolidBounds.of(uprightBounds));
  }

  @Test
  void blueHubCornerAt45DegreesDoesNotSpinOutOrTunnel() {
    double[] hubBounds = bounds(FIELD.getBlueHub());
    Pose2d start = new Pose2d(hubBounds[0] - 2.0, hubBounds[2] - 2.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);
    double diag = APPROACH_SPEED / Math.sqrt(2);
    ChassisSpeeds commanded = new ChassisSpeeds(diag, diag, 0);

    driveAndAssertNoHeadingDrift(physics, commanded, SolidBounds.of(hubBounds));
  }
}
