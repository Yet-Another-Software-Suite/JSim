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
   * @return Transformed {@link ChassisSpeeds} after physical constraints are applied.
   */
  ChassisSpeeds process(Pose2d currentPose, ChassisSpeeds inputSpeeds, Translation2d robotDimensions);
}