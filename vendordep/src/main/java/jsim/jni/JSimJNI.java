// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.jni;

import java.util.concurrent.atomic.AtomicBoolean;

/** JNI entry points for the vendor physics driver. */
public class JSimJNI {
  private JSimJNI() {}

  static boolean libraryLoaded = false;

  /** Configures whether the native library is loaded during static initialization. */
  public static class Helper {
    private Helper() {}

    private static final AtomicBoolean extractOnStaticLoad = new AtomicBoolean(true);

    /**
     * Returns whether the driver loads during static initialization.
     *
     * @return true when the driver loads on static initialization
     */
    public static boolean getExtractOnStaticLoad() {
      return extractOnStaticLoad.get();
    }

    /**
     * Sets whether the driver loads during static initialization.
     *
     * @param load the new value
     */
    public static void setExtractOnStaticLoad(boolean load) {
      extractOnStaticLoad.set(load);
    }
  }

  static {
    if (Helper.getExtractOnStaticLoad()) {
      System.loadLibrary("JSimDriver");
      libraryLoaded = true;
    }
  }

  /** Forces the native library to load. */
  public static synchronized void forceLoad() {
    if (libraryLoaded) {
      return;
    }
    System.loadLibrary("JSimDriver");
    libraryLoaded = true;
  }

  /**
   * Initializes the native driver.
   *
   * @return the value returned by the driver
   */
  public static native int initialize();

  /**
   * Creates a native world handle.
   *
   * @param fixedDtSeconds the fixed simulation timestep in seconds
   * @param enableGravity true to enable gravity for the world
   * @return the native world handle
   */
  public static native long createWorld(double fixedDtSeconds, boolean enableGravity);

  /**
   * Destroys a native world handle.
   *
   * @param worldHandle the native world handle to destroy
   */
  public static native void destroyWorld(long worldHandle);

  /**
   * Creates a body in the given world and returns its native index.
   *
   * @param worldHandle the native world handle
   * @param massKg the body mass in kilograms
   * @return the native body index
   */
  public static native int createBody(long worldHandle, double massKg);

  /**
   * Creates a generic gamepiece using the legacy ball-compatible defaults.
   *
   * @param worldHandle the native world handle
   * @return the native gamepiece index
   */
  public static native int createGamepiece(long worldHandle);

  /**
   * Creates a generic gamepiece with a spherical hitbox in the world.
   *
   * @param worldHandle the native world handle
   * @param radiusMeters sphere hitbox radius in meters
   * @param massKg gamepiece mass in kilograms
   * @param restitution coefficient of restitution in [0, 1]
   * @return the native gamepiece index
   */
  public static native int createGamepiece(
      long worldHandle, double radiusMeters, double massKg, double restitution);

    /**
     * Creates a generic gamepiece with explicit type tag.
     * @param worldHandle native world handle
     * @param type ordinal value from GamePieceType
     * @param radiusMeters sphere radius in meters
     * @param massKg mass in kilograms
     * @param restitution coefficient of restitution
     * @return native gamepiece index
     */
  public static native int createGamepieceWithType(
      long worldHandle, int type, double radiusMeters, double massKg, double restitution);

      /**
       * Creates a generic gamepiece with a string type name.
       * @param worldHandle native world handle
       * @param typeName human readable type name (e.g. "generic_sphere")
       * @param radiusMeters sphere radius in meters
       * @param massKg mass in kilograms
       * @param restitution coefficient of restitution
       * @return native gamepiece index
       */
  public static native int createGamepieceWithTypeName(
      long worldHandle, String typeName, double radiusMeters, double massKg, double restitution);

      /**
       * Reads the registered type name for a gamepiece.
       * @param worldHandle native world handle
       * @param gamepieceIndex native gamepiece index
       * @return UTF-8 type name or null when unknown
       */
  public static native String getGamepieceTypeName(long worldHandle, int gamepieceIndex);

  /**
   * Sets a body's position in meters.
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param xMeters the x position in meters
   * @param yMeters the y position in meters
   * @param zMeters the z position in meters
   * @return zero on success
   */
  public static native int setBodyPosition(
      long worldHandle, int bodyIndex, double xMeters, double yMeters, double zMeters);

  /**
   * Sets a body's linear velocity in meters per second.
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param vxMps the x velocity in meters per second
   * @param vyMps the y velocity in meters per second
   * @param vzMps the z velocity in meters per second
   * @return zero on success
   */
  public static native int setBodyLinearVelocity(
      long worldHandle, int bodyIndex, double vxMps, double vyMps, double vzMps);

    /**
     * Sets a body's world-space orientation as a quaternion.
     *
     * @param worldHandle the native world handle
     * @param bodyIndex the native body index
     * @param qw quaternion w component
     * @param qx quaternion x component
     * @param qy quaternion y component
     * @param qz quaternion z component
     * @return zero on success
     */
    public static native int setBodyOrientation(
      long worldHandle, int bodyIndex, double qw, double qx, double qy, double qz);

  /**
   * Enables or disables gravity for a body.
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param enabled true to enable gravity, false to disable it
   * @return zero on success
   */
  public static native int setBodyGravityEnabled(long worldHandle, int bodyIndex, boolean enabled);

  /**
   * Sets per-body material contact properties.
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param restitution coefficient of restitution [0, 1]
   * @param frictionKinetic kinetic friction coefficient
   * @param frictionStatic static friction coefficient
   * @param collisionDamping damping coefficient [0, 1]
   * @return zero on success
   */
  public static native int setBodyMaterial(
      long worldHandle,
      int bodyIndex,
      double restitution,
      double frictionKinetic,
      double frictionStatic,
      double collisionDamping);

  /**
   * Sets numeric material identifier used by world material-interaction tables.
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param materialId numeric material id
   * @return zero on success
   */
  public static native int setBodyMaterialId(long worldHandle, int bodyIndex, int materialId);

  /**
   * Sets collision layer and mask filters for this body.
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param collisionLayerBits body layer bitmask
   * @param collisionMaskBits body mask bitmask
   * @return zero on success
   */
  public static native int setBodyCollisionFilter(
      long worldHandle, int bodyIndex, int collisionLayerBits, int collisionMaskBits);

  /**
   * Assigns sphere aerodynamic geometry to a body (useful for game pieces from CAD).
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param radiusMeters sphere radius in meters
   * @param dragCoefficient drag coefficient (dimensionless)
   * @return zero on success
   */
  public static native int setBodyAerodynamicSphere(
      long worldHandle, int bodyIndex, double radiusMeters, double dragCoefficient);

  /**
   * Assigns box aerodynamic geometry to a body (useful for robot CAD components).
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param xMeters box x dimension in meters
   * @param yMeters box y dimension in meters
   * @param zMeters box z dimension in meters
   * @param dragCoefficient drag coefficient (dimensionless)
   * @return zero on success
   */
  public static native int setBodyAerodynamicBox(
      long worldHandle,
      int bodyIndex,
      double xMeters,
      double yMeters,
      double zMeters,
      double dragCoefficient);

    /**
     * Attempts to pick up a gamepiece into a carrier location.
     *
     * @param worldHandle the native world handle
     * @param gamepieceIndex the native gamepiece index
     * @param intakeX world-space intake x coordinate
     * @param intakeY world-space intake y coordinate
     * @param intakeZ world-space intake z coordinate
     * @param captureRadius capture radius in meters
     * @param carryOffsetX carry offset x in meters
     * @param carryOffsetY carry offset y in meters
     * @param carryOffsetZ carry offset z in meters
     * @return zero on success, non-zero on failure
     */
    public static native int pickGamepiece(
      long worldHandle,
      int gamepieceIndex,
      double intakeX,
      double intakeY,
      double intakeZ,
      double captureRadius,
      double carryOffsetX,
      double carryOffsetY,
      double carryOffsetZ);

    /**
     * Places a gamepiece at the given world position and marks it grounded.
     *
     * @param worldHandle the native world handle
     * @param gamepieceIndex the native gamepiece index
     * @param xMeters x position in meters
     * @param yMeters y position in meters
     * @param zMeters z position in meters
     * @return zero on success, non-zero on failure
     */
    public static native int placeGamepiece(long worldHandle, int gamepieceIndex,
      double xMeters, double yMeters, double zMeters);

    /**
     * Launches (outtakes) a gamepiece with the specified position and velocity.
     *
     * @param worldHandle the native world handle
     * @param gamepieceIndex the native gamepiece index
     * @param px launch position x in meters
     * @param py launch position y in meters
     * @param pz launch position z in meters
     * @param vx launch velocity x in m/s
     * @param vy launch velocity y in m/s
     * @param vz launch velocity z in m/s
     * @return zero on success, non-zero on failure
     */
    public static native int outtakeGamepiece(long worldHandle, int gamepieceIndex,
      double px, double py, double pz, double vx, double vy, double vz);

    /**
     * Native: set gamepiece position.
     *
     * @param worldHandle the native world handle
     * @param gamepieceIndex the native gamepiece index
     * @param xMeters x position in meters
     * @param yMeters y position in meters
     * @param zMeters z position in meters
     * @return zero on success, non-zero on failure
     */
  public static native int setGamepiecePosition(
      long worldHandle, int gamepieceIndex, double xMeters, double yMeters, double zMeters);

    /**
     * Native: set gamepiece linear velocity.
     *
     * @param worldHandle the native world handle
     * @param gamepieceIndex the native gamepiece index
     * @param vxMps x velocity in meters per second
     * @param vyMps y velocity in meters per second
     * @param vzMps z velocity in meters per second
     * @return zero on success, non-zero on failure
     */
  public static native int setGamepieceLinearVelocity(
      long worldHandle, int gamepieceIndex, double vxMps, double vyMps, double vzMps);

    /**
     * Native: get gamepiece position.
     *
     * @param worldHandle the native world handle
     * @param gamepieceIndex the native gamepiece index
     * @param outXyzMeters output array of length 3 receiving x,y,z in meters
     * @return zero on success, non-zero on failure
     */
  public static native int getGamepiecePosition(
      long worldHandle, int gamepieceIndex, double[] outXyzMeters);

    /**
     * Native: get gamepiece linear velocity.
     *
     * @param worldHandle the native world handle
     * @param gamepieceIndex the native gamepiece index
     * @param outVxyzMps output array of length 3 receiving vx,vy,vz in m/s
     * @return zero on success, non-zero on failure
     */
  public static native int getGamepieceLinearVelocity(
      long worldHandle, int gamepieceIndex, double[] outVxyzMps);

    // Deprecated ball-named methods removed; use gamepiece equivalents.

  /**
   * Sets the world's gravity vector in meters per second squared.
   *
   * @param worldHandle the native world handle
   * @param gxMps2 the x gravity component in meters per second squared
   * @param gyMps2 the y gravity component in meters per second squared
   * @param gzMps2 the z gravity component in meters per second squared
   * @return zero on success
   */
  public static native int setWorldGravity(
      long worldHandle, double gxMps2, double gyMps2, double gzMps2);

  /**
   * Configures world-level aerodynamics parameters.
   *
   * @param worldHandle the native world handle
   * @param enabled true to enable aerodynamics
   * @param airDensityKgPm3 air density in kg/m^3
   * @param linearDragNPerMps linear drag coefficient in N/(m/s)
   * @param magnusCoefficient Magnus coefficient
   * @param defaultDragCoefficient default drag coefficient
   * @param defaultReferenceAreaM2 default drag reference area in m^2
   * @return zero on success
   */
  public static native int setWorldAerodynamics(
      long worldHandle,
      boolean enabled,
      double airDensityKgPm3,
      double linearDragNPerMps,
      double magnusCoefficient,
      double defaultDragCoefficient,
      double defaultReferenceAreaM2);

  /**
   * Sets per-material-pair contact interaction overrides.
   *
   * @param worldHandle the native world handle
   * @param materialAId first material id
   * @param materialBId second material id
   * @param restitution override restitution [0, 1]
   * @param friction override kinetic friction coefficient
   * @param enabled true to enable this pair override
   * @return zero on success
   */
  public static native int setMaterialInteraction(
      long worldHandle,
      int materialAId,
      int materialBId,
      double restitution,
      double friction,
      boolean enabled);

  /**
   * Advances the world by the given number of steps.
   *
   * @param worldHandle the native world handle
   * @param steps the number of steps to advance
   * @return zero on success
   */
  public static native int stepWorld(long worldHandle, int steps);

  /**
   * Reads a body's position into {@code outXyzMeters}.
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param outXyzMeters the output array that receives the position
   * @return zero on success
   */
  public static native int getBodyPosition(long worldHandle, int bodyIndex, double[] outXyzMeters);

  /**
   * Reads a body's linear velocity into {@code outVxyzMps}.
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param outVxyzMps the output array that receives the linear velocity
   * @return zero on success
   */
  public static native int getBodyLinearVelocity(
      long worldHandle, int bodyIndex, double[] outVxyzMps);

    /**
     * Reads a body's orientation as a quaternion into {@code outWxyz}.
     *
     * @param worldHandle the native world handle
     * @param bodyIndex the native body index
     * @param outWxyz the output array that receives qw,qx,qy,qz
     * @return zero on success
     */
    public static native int getBodyOrientation(
      long worldHandle, int bodyIndex, double[] outWxyz);

  /**
   * Exports flattened body poses as [x, y, z, qw, qx, qy, qz] blocks.
   *
   * @param worldHandle the native world handle
   * @param outPose7 the destination array sized for N*7 entries
   * @return the number of body blocks written, or negative on error
   */
  public static native int getBodyPose7Array(long worldHandle, double[] outPose7);

  /**
   * Exports flattened body velocities as [vx, vy, vz, wx, wy, wz] blocks.
   *
   * @param worldHandle the native world handle
   * @param outVelocity6 the destination array sized for N*6 entries
   * @return the number of body blocks written, or negative on error
   */
  public static native int getBodyVelocity6Array(long worldHandle, double[] outVelocity6);

  /**
   * Exports flattened full body state as
   * [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz] blocks.
   *
   * @param worldHandle the native world handle
   * @param outState13 the destination array sized for N*13 entries
   * @return the number of body blocks written, or negative on error
   */
  public static native int getBodyState13Array(long worldHandle, double[] outState13);

    // Deprecated ball-named getters removed; use gamepiece equivalents.
}
