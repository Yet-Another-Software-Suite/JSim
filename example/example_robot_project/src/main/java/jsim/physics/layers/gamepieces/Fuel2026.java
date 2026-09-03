package jsim.physics.layers.gamepieces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * A single FUEL game piece: a 6in sphere tracked as a point mass in three dimensions.
 *
 * <p>FUEL is simulated as a free particle rather than a rigid body -- it has a position and a
 * velocity but no orientation, since a smooth sphere's spin doesn't affect anything else in the
 * model. {@code jsim.physics.layers.FuelLayer} owns the integration and all collision
 * response; this class is just the state those steps read and write.
 */
public class Fuel2026 {

  private Translation3d position;
  private Translation3d velocity;
  private boolean supported;
  private boolean intaked;

  /**
   * Creates a FUEL piece at rest.
   *
   * @param position Field-relative position of the ball's center, in meters.
   */
  public Fuel2026(Translation3d position) {
    this(position, new Translation3d());
  }

  /**
   * Creates a moving FUEL piece.
   *
   * @param position Field-relative position of the ball's center, in meters.
   * @param velocity Field-relative velocity, in meters per second.
   */
  public Fuel2026(Translation3d position, Translation3d velocity) {
    this.position = position;
    this.velocity = velocity;
  }

  /** Field-relative position of this ball's center, in meters. */
  public Translation3d getPosition() {
    return position;
  }

  public void setPosition(Translation3d position) {
    this.position = position;
  }

  /** Field-relative velocity of this ball, in meters per second. */
  public Translation3d getVelocity() {
    return velocity;
  }

  public void setVelocity(Translation3d velocity) {
    this.velocity = velocity;
  }

  /** Adds a velocity change to this ball, e.g. from a collision impulse. */
  public void addImpulse(Translation3d impulse) {
    this.velocity = velocity.plus(impulse);
  }

  /** Moves this ball by {@code offset} without changing its velocity, e.g. to resolve an overlap. */
  public void translate(Translation3d offset) {
    this.position = position.plus(offset);
  }

  /**
   * Whether this ball is currently resting on a surface (the carpet, a BUMP, a TRENCH, ...).
   * Supported balls skip gravity and lose horizontal speed to rolling friction instead of
   * jittering against whatever they are sitting on.
   */
  public boolean isSupported() {
    return supported;
  }

  public void setSupported(boolean supported) {
    this.supported = supported;
  }

  /**
   * Whether this ball has been picked up by a registered intake. Intaked FUEL is removed from the
   * simulation, so this flag only matters to callers still holding a reference to the piece.
   */
  public boolean isIntaked() {
    return intaked;
  }

  public void setIntaked(boolean intaked) {
    this.intaked = intaked;
  }

  /** This ball's position projected onto the carpet, for robot-relative geometry. */
  public Translation2d getTranslation2d() {
    return position.toTranslation2d();
  }

  /** This ball's position as an (unrotated) {@link Pose3d}, for NetworkTables publishing. */
  public Pose3d getPose3d() {
    return new Pose3d(position, new Rotation3d());
  }
}
