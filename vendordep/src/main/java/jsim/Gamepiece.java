// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import jsim.api.GamePieceType;

/**
 * Generic handle for a gamepiece managed by {@link PhysicsWorld}.
 *
 * <p>Usage example:
 * <pre>{@code
 * // Spawn a ball gamepiece on the field:
 * Gamepiece ball = world.createGamepiece(GamePieceType.BALL);
 * ball.setPosition(new Translation3d(5.0, 3.0, 0.25));
 *
 * // Intake: attempt pick-up from intake position
 * Vec3 carryOffset = new Vec3(0.0, 0.0, 0.1);
 * boolean picked = ball.pick(new Pose3d(5.0, 3.0, 0.25, new Rotation3d()), 0.3, carryOffset);
 *
 * // Shoot: launch from shooter position
 * ball.outtake(
 *     new Pose3d(2.0, 4.0, 0.9, new Rotation3d()),
 *     new LinearVelocity3d(10.0, 0.0, 5.0)
 * );
 * }</pre>
 */
public class Gamepiece {
  /** The owning physics world for this handle. */
  protected final PhysicsWorld world;

  /** Native index for this gamepiece in the driver. */
  protected final int gamepieceIndex;

  /** Declared gamepiece type when created from a typed API. */
  protected final GamePieceType type;

  Gamepiece(PhysicsWorld world, int gamepieceIndex) {
    this(world, gamepieceIndex, null);
  }

  Gamepiece(PhysicsWorld world, int gamepieceIndex, GamePieceType type) {
    this.world = world;
    this.gamepieceIndex = gamepieceIndex;
    this.type = type;
  }

  /**
   * Gets the native gamepiece index for this handle.
   *
   * @return the native gamepiece index
   */
  public int gamepieceIndex() {
    return gamepieceIndex;
  }

  /**
   * Gets the declared gamepiece type when created from a typed API.
   *
   * @return the declared gamepiece type, or null when unknown
   */
  public GamePieceType type() {
    return type;
  }

  /**
   * Sets the gamepiece world-space position in meters.
   *
   * @param pose the new position (only translation component is used)
   */
  public void setPosition(Pose3d pose) {
    Translation3d translation = pose.getTranslation();
    world.setGamepiecePosition(
        gamepieceIndex, translation.getX(), translation.getY(), translation.getZ());
  }

  /**
   * Sets the gamepiece world-space position in meters.
   *
   * @param positionMeters the new position in meters
   */
  public void setPosition(Translation3d positionMeters) {
    world.setGamepiecePosition(
        gamepieceIndex,
        positionMeters.getX(),
        positionMeters.getY(),
        positionMeters.getZ());
  }

  /**
   * Sets the gamepiece world-space position in meters.
   *
   * @param positionMeters the new position in meters
   */
  public void setPosition(Vec3 positionMeters) {
    world.setGamepiecePosition(gamepieceIndex, positionMeters.x(), positionMeters.y(), positionMeters.z());
  }

  /**
   * Sets the gamepiece world-space position.
   *
   * @param x x position
   * @param y y position
   * @param z z position
   */
  public void setPosition(Distance x, Distance y, Distance z) {
    world.setGamepiecePosition(gamepieceIndex, x.in(Meters), y.in(Meters), z.in(Meters));
  }

  /**
   * Sets the gamepiece world-space linear velocity in meters per second.
   *
   * @param velocityMps the new linear velocity in meters per second
   */
  public void setLinearVelocity(LinearVelocity3d velocityMps) {
    world.setGamepieceLinearVelocity(
        gamepieceIndex,
        velocityMps.getVxMetersPerSecond(),
        velocityMps.getVyMetersPerSecond(),
        velocityMps.getVzMetersPerSecond());
  }

  /**
   * Sets the gamepiece world-space linear velocity in meters per second.
   *
   * @param velocityMps the new linear velocity in meters per second
   */
  public void setLinearVelocity(Vec3 velocityMps) {
    world.setGamepieceLinearVelocity(gamepieceIndex, velocityMps.x(), velocityMps.y(), velocityMps.z());
  }

  /**
   * Sets the gamepiece world-space linear velocity in meters per second.
   *
   * @param vxMetersPerSecond x velocity in meters per second
   * @param vyMetersPerSecond y velocity in meters per second
   * @param vzMetersPerSecond z velocity in meters per second
   */
  public void setLinearVelocity(
      double vxMetersPerSecond, double vyMetersPerSecond, double vzMetersPerSecond) {
    world.setGamepieceLinearVelocity(
        gamepieceIndex, vxMetersPerSecond, vyMetersPerSecond, vzMetersPerSecond);
  }

  /**
   * Convenience method to set both position and launch velocity.
   *
   * @param pose launch position (only translation component is used)
   * @param velocityMps launch velocity in meters per second
   */
  public void outtake(Pose3d pose, LinearVelocity3d velocityMps) {
    world.outtakeGamepiece(gamepieceIndex(),
      pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getTranslation().getZ(),
      velocityMps.getVxMetersPerSecond(), velocityMps.getVyMetersPerSecond(), velocityMps.getVzMetersPerSecond());
  }

  /**
   * Convenience method to set both position and launch velocity.
   *
   * @param positionMeters launch position in meters
   * @param velocityMps launch velocity in meters per second
   */
  public void outtake(Translation3d positionMeters, LinearVelocity3d velocityMps) {
    world.outtakeGamepiece(gamepieceIndex(), positionMeters.getX(), positionMeters.getY(), positionMeters.getZ(),
      velocityMps.getVxMetersPerSecond(), velocityMps.getVyMetersPerSecond(), velocityMps.getVzMetersPerSecond());
  }

  /**
   * Convenience method to set both position and launch velocity.
   *
   * @param positionMeters launch position in meters
   * @param velocityMps launch velocity in meters per second
   */
  public void outtake(Vec3 positionMeters, Vec3 velocityMps) {
    world.outtakeGamepiece(gamepieceIndex(), positionMeters.x(), positionMeters.y(), positionMeters.z(),
      velocityMps.x(), velocityMps.y(), velocityMps.z());
  }

  // Deprecated compatibility alias removed; use `outtake(...)` instead.

  /**
   * Attempt to pick this gamepiece into a carrier.
   * @param intakePos intake world position
   * @param captureRadius capture radius in meters
   * @param carryOffset offset from intake to carried resting position
   * @return true when pickup succeeded
   */
  public boolean pick(Pose3d intakePos, double captureRadius, Vec3 carryOffset) {
    Translation3d t = intakePos.getTranslation();
    int rc = world.pickGamepiece(gamepieceIndex,
        t.getX(), t.getY(), t.getZ(),
        captureRadius,
        carryOffset.x(), carryOffset.y(), carryOffset.z());
    return rc == 0;
  }

  /**
   * Place this gamepiece at the given world position and mark grounded.
   *
   * @param pos world-space position in meters
   */
  public void place(Translation3d pos) {
    world.placeGamepiece(gamepieceIndex, pos.getX(), pos.getY(), pos.getZ());
  }

  /**
   * Gets the current world-space position in meters.
   *
   * @return the gamepiece position
   */
  public Pose3d position() {
    return world.getGamepiecePosition(gamepieceIndex);
  }

  /**
   * Gets the current world-space linear velocity in meters per second.
   *
   * @return the gamepiece linear velocity
   */
  public LinearVelocity3d linearVelocity() {
    return world.getGamepieceLinearVelocity(gamepieceIndex);
  }

  /**
   * Returns the registered type name for this gamepiece, if any.
   * @return human-readable type name or null
   */
  public String typeName() {
    if (type != null) {
      return type.name().toLowerCase();
    }
    return world.getGamepieceTypeName(gamepieceIndex);
  }

  /**
   * Convenience: is this gamepiece a ball (default spherical physics)?
   *
   * @return true when this gamepiece should be treated as a ball/sphere
   */
  public boolean isBall() {
    String t = typeName();
    if (t == null || t.isEmpty()) return true;
    String lower = t.toLowerCase();
    return lower.contains("ball") || lower.contains("sphere") || lower.startsWith("generic_");
  }
}
