package jsim.physics.layers.gamepieces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Common state for a simulated game piece.
 *
 * <p>The base class intentionally knows nothing about a piece's shape or physics response. It
 * provides the state and lifecycle helpers shared by pieces that can be spawned, intaked, or held
 * by a robot. Concrete pieces can add their own collision and integration behavior.
 */
public class Gamepiece {

  protected Translation3d position;
  protected Translation3d velocity;
  private boolean supported;
  private boolean intaked;
  private Transform3d robotRelativeAttachment;

  /** Creates a game piece at rest. */
  public Gamepiece(Translation3d position) {
    this(position, new Translation3d());
  }

  /** Creates a moving game piece. */
  public Gamepiece(Translation3d position, Translation3d velocity) {
    this.position = position;
    this.velocity = velocity;
  }

  /** Returns the field-relative position of this piece's center. */
  public Translation3d getPosition() {
    return position;
  }

  /** Sets this piece's field-relative position. */
  public void setPosition(Translation3d position) {
    this.position = position;
  }

  /** Returns this piece's field-relative velocity. */
  public Translation3d getVelocity() {
    return velocity;
  }

  /** Sets this piece's field-relative velocity. */
  public void setVelocity(Translation3d velocity) {
    this.velocity = velocity;
  }

  /** Adds a velocity change to this piece. */
  public void addImpulse(Translation3d impulse) {
    this.velocity = velocity.plus(impulse);
  }

  /** Moves this piece without changing its velocity. */
  public void translate(Translation3d offset) {
    this.position = position.plus(offset);
  }

  /** Returns whether this piece is resting on a supporting surface. */
  public boolean isSupported() {
    return supported;
  }

  /** Sets whether this piece is resting on a supporting surface. */
  public void setSupported(boolean supported) {
    this.supported = supported;
  }

  /** Returns whether this piece has been picked up by an intake. */
  public boolean isIntaked() {
    return intaked;
  }

  /** Sets whether this piece has been picked up by an intake. */
  public void setIntaked(boolean intaked) {
    this.intaked = intaked;
  }

  /** Returns this piece's position projected onto the carpet. */
  public Translation2d getTranslation2d() {
    return position.toTranslation2d();
  }

  /** Returns this piece's position as an unrotated pose. */
  public Pose3d getPose3d() {
    return new Pose3d(position, new Rotation3d());
  }

  /**
   * Attaches this piece to a robot at a robot-relative transform.
   *
   * <p>The piece immediately follows the supplied robot pose after
   * {@link #updateRobotAttachment(Pose2d)} is called. Attaching clears its velocity and support
   * state because it is no longer an independent physics body.
   */
  public void attachToRobot(Pose2d robotPose, Transform3d robotRelativeTransform) {
    robotRelativeAttachment = robotRelativeTransform;
    velocity = new Translation3d();
    supported = false;
    updateRobotAttachment(robotPose);
  }

  /** Updates the position of an attached piece from the robot's latest pose. */
  public void updateRobotAttachment(Pose2d robotPose) {
    if (robotRelativeAttachment == null) {
      throw new IllegalStateException("Cannot update a game piece that is not attached to a robot.");
    }
    position = new Pose3d(robotPose).plus(robotRelativeAttachment).getTranslation();
  }

  /** Detaches this piece so it can move independently again. */
  public void detachFromRobot() {
    robotRelativeAttachment = null;
  }

  /** Returns whether this piece is currently attached to a robot. */
  public boolean isAttachedToRobot() {
    return robotRelativeAttachment != null;
  }
}