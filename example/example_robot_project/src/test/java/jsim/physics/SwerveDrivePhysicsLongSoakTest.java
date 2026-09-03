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
 * Regression test proving a robot held continuously against an obstacle does not slowly creep
 * through it given enough real time -- as opposed to the ~5-10s soaks the other collision tests
 * run, which are too short to reveal a *slow* net drift (a few mm/s can still cover a couple
 * meters over several minutes of real play, easily enough to end up on the far side of a HUB).
 *
 * <p>Root cause this guards against: {@link Dyn4jCollisionLayer#process} used to report the raw
 * post-solve velocity dyn4j's contact solver produced, then let the caller ({@link
 * SwerveDrivePhysics}) re-integrate pose from that velocity separately over the same dt. But
 * {@code world.update(dt)} also applies its own internal position-correction pass on top of
 * velocity resolution, and since this body's transform gets externally re-synced to the caller's
 * own recomputed pose every call (rather than persisting continuously like a normal dyn4j body),
 * that correction was silently discarded every frame -- opening the door to exactly this kind of
 * slow accumulating creep. The fix derives the reported velocity from dyn4j's actual resulting
 * displacement instead, so the caller's own re-integration reproduces what dyn4j already resolved
 * (position correction included) rather than redoing a naive, uncorrected integration.
 */
class SwerveDrivePhysicsLongSoakTest {

  private static final Field2026 FIELD = new Field2026();
  private static final double ROBOT_SIZE = 0.9;
  private static final double ROBOT_HALF = ROBOT_SIZE / 2.0;
  private static final double APPROACH_SPEED = 4.0; // m/s
  // 5 simulated minutes -- long enough that even a slow, "stable-looking" mm/s creep would have
  // covered meters by the end, while a genuinely held robot stays put regardless of duration.
  private static final int STEPS = 15_000;

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

  @Test
  void robotHeldAgainstHubDoesNotCreepThroughItOverFiveMinutes() {
    double[] hubBounds = bounds(FIELD.getBlueHub());
    double approachY = (hubBounds[2] + hubBounds[3]) / 2.0;
    Pose2d start = new Pose2d(hubBounds[1] + 2.0, approachY, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);
    ChassisSpeeds commanded = new ChassisSpeeds(-APPROACH_SPEED, 0, 0);
    SwerveModulePosition[] positions = zeroModulePositions();

    // Give the robot time to close the initial ~2m approach and settle into contact before the
    // creep window starts -- otherwise the approach itself (a legitimate multi-meter move) gets
    // counted as "drift", masking any real post-settling creep entirely.
    int settlingWindowStart = 200;
    double maxXAfterSettling = Double.NEGATIVE_INFINITY;
    double minXAfterSettling = Double.POSITIVE_INFINITY;
    Pose2d pose = start;
    for (int i = 0; i < STEPS; i++) {
      pose = physics.update(commanded, Rotation2d.kZero, positions, Dyn4jCollisionLayer.DEFAULT_DT_SECONDS).pose();
      if (i > settlingWindowStart) {
        maxXAfterSettling = Math.max(maxXAfterSettling, pose.getX());
        minXAfterSettling = Math.min(minXAfterSettling, pose.getX());
      }
    }

    double hubFarEdgeX = hubBounds[0]; // the far (min-X) face -- tunneling all the way through
    assertTrue(pose.getX() > hubFarEdgeX + ROBOT_HALF,
        "Robot crept all the way through the HUB over the soak: final x=" + pose.getX());

    double driftAfterSettling = maxXAfterSettling - minXAfterSettling;
    assertTrue(driftAfterSettling < 0.3,
        "Robot drifted " + driftAfterSettling + "m into the HUB over a 5-minute-equivalent soak "
            + "(from x=" + maxXAfterSettling + " down to x=" + minXAfterSettling + "); expected it to "
            + "stay held at the contact point regardless of how long it's pushed.");
  }
}
