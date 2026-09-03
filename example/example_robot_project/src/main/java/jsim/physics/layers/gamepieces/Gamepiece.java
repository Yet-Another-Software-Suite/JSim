package jsim.physics.layers.gamepieces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Common state and lifecycle operations for a simulated game piece.
 *
 * <p>The base class intentionally knows nothing about a piece's shape, mass, or physics response.
 * It provides the state and lifecycle helpers shared by pieces that can be spawned, intaked, or
 * held by a robot. Concrete pieces should extend this class and add their own collision and
 * integration behavior.
 *
 * <p>Pieces that are picked up can be attached to a robot with a robot-relative {@link Transform3d}.
 * The owning simulation is responsible for calling {@link #updateRobotAttachment(Pose2d)} as the
 * robot moves, and for calling {@link #detachFromRobot()} when the piece is released.
 */
public class Gamepiece {

  /** Field-relative center position, in meters. */
  protected Translation3d position;

  /** Field-relative velocity, in meters per second. */
  protected Translation3d velocity;
  private boolean supported;
  private boolean intaked;
  private Transform3d robotRelativeAttachment;

  /**
   * Creates a game piece at rest.
   *
   * @param position Field-relative position of the piece's center, in meters.
   */
  public Gamepiece(Translation3d position) {
    this(position, new Translation3d());
  }

  /**
   * Creates a moving game piece.
   *
   * @param position Field-relative position of the piece's center, in meters.
   * @param velocity Field-relative velocity of the piece, in meters per second.
   */
  public Gamepiece(Translation3d position, Translation3d velocity) {
    this.position = position;
    this.velocity = velocity;
  }

  /**
   * Returns the field-relative position of this piece's center.
   *
   * @return Field-relative center position, in meters.
   */
  public Translation3d getPosition() {
    return position;
  }

  /**
   * Sets this piece's field-relative position.
   *
   * @param position New field-relative center position, in meters.
   */
  public void setPosition(Translation3d position) {
    this.position = position;
  }

  /**
   * Returns this piece's field-relative velocity.
   *
   * @return Field-relative velocity, in meters per second.
   */
  public Translation3d getVelocity() {
    return velocity;
  }

  /**
   * Sets this piece's field-relative velocity.
   *
   * @param velocity New field-relative velocity, in meters per second.
   */
  public void setVelocity(Translation3d velocity) {
    this.velocity = velocity;
  }

  /**
   * Adds a velocity change to this piece.
   *
   * @param impulse Velocity change to add, in meters per second.
   */
  public void addImpulse(Translation3d impulse) {
    this.velocity = velocity.plus(impulse);
  }

  /**
   * Moves this piece without changing its velocity.
   *
   * @param offset Field-relative displacement, in meters.
   */
  public void translate(Translation3d offset) {
    this.position = position.plus(offset);
  }

  /**
   * Returns whether this piece is resting on a supporting surface.
   *
   * @return Whether this piece is supported.
   */
  public boolean isSupported() {
    return supported;
  }

  /**
   * Sets whether this piece is resting on a supporting surface.
   *
   * @param supported Whether this piece is supported.
   */
  public void setSupported(boolean supported) {
    this.supported = supported;
  }

  /**
   * Returns whether this piece has been picked up by an intake.
   *
   * @return Whether this piece has been intaked.
   */
  public boolean isIntaked() {
    return intaked;
  }

  /**
   * Sets whether this piece has been picked up by an intake.
   *
   * @param intaked Whether this piece has been intaked.
   */
  public void setIntaked(boolean intaked) {
    this.intaked = intaked;
  }

  /**
   * Returns this piece's position projected onto the carpet.
   *
   * @return Field-relative 2D center position, in meters.
   */
  public Translation2d getTranslation2d() {
    return position.toTranslation2d();
  }

  /**
   * Returns this piece's position as an unrotated pose.
   *
   * @return Field-relative pose of this piece.
   */
  public Pose3d getPose3d() {
    return new Pose3d(position, new Rotation3d());
  }

  /**
   * Attaches this piece to a robot at a robot-relative transform.
   *
  * <p>The piece is positioned immediately using {@code robotPose}, then follows subsequent robot
  * poses when {@link #updateRobotAttachment(Pose2d)} is called. Attaching clears its velocity and
  * support state because it is no longer an independent physics body.
  *
  * @param robotPose Current field-relative robot pose.
  * @param robotRelativeTransform Position and orientation of the piece relative to the robot.
   */
  public void attachToRobot(Pose2d robotPose, Transform3d robotRelativeTransform) {
    robotRelativeAttachment = robotRelativeTransform;
    velocity = new Translation3d();
    supported = false;
    updateRobotAttachment(robotPose);
  }

  /**
   * Updates the position of an attached piece from the robot's latest pose.
   *
   * @param robotPose Current field-relative robot pose.
   * @throws IllegalStateException if this piece is not attached to a robot.
   */
  public void updateRobotAttachment(Pose2d robotPose) {
    if (robotRelativeAttachment == null) {
      throw new IllegalStateException("Cannot update a game piece that is not attached to a robot.");
    }
    position = new Pose3d(robotPose).plus(robotRelativeAttachment).getTranslation();
  }

  /**
   * Detaches this piece so it can move independently again.
   *
   * <p>Detaching does not add the robot's velocity to the piece. A shooter or other mechanism can
   * set the desired release velocity with {@link #setVelocity(Translation3d)} afterward.
   */
  public void detachFromRobot() {
    robotRelativeAttachment = null;
  }

  /**
   * Returns whether this piece is currently attached to a robot.
   *
   * @return Whether this piece has an active robot-relative attachment.
   */
  public boolean isAttachedToRobot() {
    return robotRelativeAttachment != null;
  }
}