package jsim.physics.layers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import jsim.physics.SwerveDrivePhysics;

/**
 * Common interface for modular simulation layers that alter or constrain chassis velocities.
 *
 * <p>Implementations process raw commanded chassis speeds sequentially through
 * {@link SwerveDrivePhysics#addLayer(PhysicsLayer)} to simulate physical phenomena such as rigid
 * wall collisions, carpet friction, or wheel traction limits.
 */
@FunctionalInterface
public interface PhysicsLayer {

  /**
   * Processes input speeds against environmental or physical constraints.
   *
   * @param currentPose Current ground-truth field position of the robot.
   * @param inputSpeeds Desired robot-relative chassis speeds before layer evaluation.
   * @param robotDimensions Half-length (X) and half-width (Y) bumper dimensions in meters.
   * @param dtSeconds Time elapsed since the previous update step, in seconds.
   * @return Transformed {@link ChassisSpeeds} after physical constraints are applied.
   */
  ChassisSpeeds process(
      Pose2d currentPose, ChassisSpeeds inputSpeeds, Translation2d robotDimensions, double dtSeconds);

  /**
   * Resets any internal simulation state this layer carries between {@link #process} calls so the
   * layer treats the robot as being at {@code pose} with {@code robotRelativeSpeeds} right now,
   * discarding prior position/velocity history (e.g. accumulated momentum from a collision).
   *
   * <p>The default implementation is a no-op, for layers (such as simple friction models) that
   * don't carry any state of their own between calls.
   *
   * @param pose Field-relative pose to reset to.
   * @param robotRelativeSpeeds Robot-relative chassis speeds to assume immediately after reset.
   */
  default void reset(Pose2d pose, ChassisSpeeds robotRelativeSpeeds) {
    // no-op by default
  }
}