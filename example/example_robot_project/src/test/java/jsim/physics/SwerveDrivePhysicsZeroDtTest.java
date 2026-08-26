package jsim.physics;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertFalse;
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
 * Regression test proving a zero-elapsed-time {@link SwerveDrivePhysics#update} call doesn't
 * permanently corrupt the ground-truth pose.
 *
 * <p>Root cause this guards against: {@link jsim.physics.layers.Dyn4jCollisionLayer#process}
 * derives an effective velocity as {@code displacement / dtSeconds}. With {@code dtSeconds == 0}
 * that's {@code 0/0}, which is NaN rather than 0. {@link Pose2d#exp} propagates a NaN twist into a
 * permanently-NaN translation, and every later call stays NaN forever regardless of dtSeconds,
 * since {@code currentPose} is the base of each subsequent exponential integration. The pose's
 * rotation is unaffected (it's independently overwritten from the real gyro angle each call), so
 * the visible symptom is a robot that keeps turning correctly but never translates again -- exactly
 * what {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} returning the same reading across two
 * consecutive {@code simulationPeriodic()} calls (a real, observed occurrence in WPILib sim, not
 * just a theoretical edge case) would trigger.
 */
class SwerveDrivePhysicsZeroDtTest {

  private static SwerveModulePosition[] zeroModulePositions() {
    return new SwerveModulePosition[] {
        new SwerveModulePosition(0, new Rotation2d()), new SwerveModulePosition(0, new Rotation2d()),
        new SwerveModulePosition(0, new Rotation2d()), new SwerveModulePosition(0, new Rotation2d())
    };
  }

  @Test
  void zeroDtStepDoesNotPermanentlyFreezeTranslation() {
    Translation2d[] moduleLocations = {
        new Translation2d(0.3, 0.3), new Translation2d(0.3, -0.3),
        new Translation2d(-0.3, 0.3), new Translation2d(-0.3, -0.3)
    };
    Pose2d start = new Pose2d(3, 3, Rotation2d.kZero);
    SwerveDrivePhysics physics = new SwerveDrivePhysics(
        moduleLocations, Meters.of(0.9), Meters.of(0.9), start, zeroModulePositions());
    physics.addLayer(new Dyn4jCollisionLayer(Kilograms.of(50.0), new Field2026()));

    ChassisSpeeds commanded = new ChassisSpeeds(2.0, 0, 0);
    SwerveModulePosition[] positions = zeroModulePositions();

    Pose2d beforeGlitch = physics.update(commanded, Rotation2d.kZero, positions, 0.02).pose();
    assertFalse(Double.isNaN(beforeGlitch.getX()), "Pose already NaN before the zero-dt tick");

    // Simulate a duplicate-timestamp tick.
    Pose2d duringGlitch = physics.update(commanded, Rotation2d.kZero, positions, 0.0).pose();
    assertFalse(Double.isNaN(duringGlitch.getX()) || Double.isNaN(duringGlitch.getY()),
        "A zero-dt step corrupted the pose to NaN: " + duringGlitch);

    Pose2d afterGlitch = physics.update(commanded, Rotation2d.kZero, positions, 0.02).pose();
    assertFalse(Double.isNaN(afterGlitch.getX()) || Double.isNaN(afterGlitch.getY()),
        "Pose stayed NaN on the step after the zero-dt tick: " + afterGlitch);
    assertTrue(afterGlitch.getX() > beforeGlitch.getX(),
        "Robot should keep translating normally after a zero-dt tick, but x didn't advance: "
            + "before=" + beforeGlitch + " after=" + afterGlitch);
  }
}
