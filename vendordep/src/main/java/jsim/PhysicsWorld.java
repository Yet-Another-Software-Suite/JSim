// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim;

import java.util.ArrayList;
import java.util.List;
import jsim.api.GamePieceType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Translation3d;
import jsim.jni.JSimJNI;

/**
 * Thin Java wrapper around the native physics world implementation.
 *
 * <p>Usage example:
 * <pre>{@code
 * PhysicsWorld world = JSim.createPhysicsWorld(0.02, true); // 20 ms timestep, gravity on
 * StateManager.getInstance().setPhysicsWorld(world);
 *
 * // Add a gamepiece
 * Gamepiece note = world.createGamepiece(GamePieceType.BALL);
 * note.setPosition(new Translation3d(5.0, 3.0, 0.25));
 *
 * // Register step listener for NT publishing
 * WorldPosePublisher pub = new WorldPosePublisher(world.getNativeHandle(), world.getMaxBodies());
 * world.addStepListener(pub::publishFrame);
 *
 * // Advance simulation (or call StateManager.stepPhysics())
 * world.step();
 * }</pre>
 */
public final class PhysicsWorld implements AutoCloseable {
	private long worldHandle;
	private final double fixedDtSeconds;
	private final List<Gamepiece> gamepieces = new ArrayList<>();
	private final List<Runnable> stepListeners = new ArrayList<>();

	/**
	 * Supported gamepiece hitbox families.
	 */
	public enum HitboxType {
		/** Spherical hitbox using ball-style physics. */
		SPHERE
	}

	/**
	 * Creates a native physics world.
	 *
	 * @param fixedDtSeconds fixed simulation timestep in seconds
	 * @param enableGravity whether gravity is enabled for newly created bodies
	 */
	public PhysicsWorld(double fixedDtSeconds, boolean enableGravity) {
		JSimJNI.forceLoad();
		this.fixedDtSeconds = fixedDtSeconds;
		this.worldHandle = JSimJNI.createWorld(fixedDtSeconds, enableGravity);
		if (worldHandle == 0) {
			throw new JSimException("Failed to create native PhysicsWorld", 0,
				"Ensure the native JSim library is available and that permissions allow loading it.");
		}
	}

	/**
	 * Returns the fixed timestep configured for this world.
	 *
	 * @return fixed timestep in seconds
	 */
	public double getFixedDtSeconds() {
		return fixedDtSeconds;
	}

	/**
	 * Returns the native world handle for interop (primarily for publishers).
	 *
	 * <p>This is intentionally provided so callers can create native-backed helpers
	 * like telemetry publishers that require the raw handle.
	 *
	 * @return native world handle (non-zero while world is valid)
	 */
	public long getNativeHandle() {
		return worldHandle;
	}


	/**
	 * Returns a reasonable default maximum number of bodies to export for telemetry.
	 *
	 * <p>Callers may use this value when constructing publishers that snapshot world state.
	 *
	 * @return default max bodies for telemetry (units: count)
	 */
	public int getMaxBodies() {
		return 128;
	}

	/**
	 * Creates a new body with the provided mass in kilograms.
	 *
	 * @param massKg the body mass in kilograms
	 * @return the created body handle
	 */
	public PhysicsBody createBody(double massKg) {
		int index = JSimJNI.createBody(worldHandle, massKg);
		if (index < 0) {
			throw new JSimException("Failed to create body with mass=" + massKg + " kg", index,
				"Body mass must be a positive, finite value. Received: " + massKg);
		}
		return new PhysicsBody(this, index);
	}

	/**
	 * Creates a new generic gamepiece in the world.
	 *
	 * <p>Current native support is spherical hitboxes. Additional hitbox types can
	 * be added without changing the top-level gamepiece abstraction.
	 *
	 * @param radiusMeters sphere radius in meters
	 * @param massKg gamepiece mass in kilograms
	 * @param restitution coefficient of restitution in [0, 1]
	 * @return the created gamepiece handle
	 */
	public Gamepiece createGamepiece(double radiusMeters, double massKg, double restitution) {
		int index = JSimJNI.createGamepiece(worldHandle, radiusMeters, massKg, restitution);
		if (index < 0) {
			throw new JSimException(
				"Failed to create gamepiece in physics world",
				index,
				"Verify hitbox and material parameters. Radius and mass must be positive finite values.");
		}

		Gamepiece gamepiece = new Gamepiece(this, index);
		gamepieces.add(gamepiece);
		return gamepiece;
	}

	/**
	 * Creates a spherical gamepiece with default physical parameters.
	 *
	 * @return the created gamepiece handle
	 */
	public Gamepiece createGamepiece() {
		return createGamepiece(0.12, 0.27, 0.45);
	}

	/**
	 * Creates a new gamepiece with an explicit type tag.
	 *
	 * @param type gamepiece type enum
	 * @param radiusMeters sphere radius in meters
	 * @param massKg mass in kilograms
	 * @param restitution coefficient of restitution
	 * @return created gamepiece
	 */
	public Gamepiece createGamepiece(GamePieceType type, double radiusMeters, double massKg, double restitution) {
		int index = JSimJNI.createGamepieceWithType(worldHandle, type.ordinal(), radiusMeters, massKg, restitution);
		if (index < 0) {
			throw new JSimException(
				"Failed to create gamepiece in physics world",
				index,
				"Verify hitbox and material parameters. Radius and mass must be positive finite values.");
		}

		Gamepiece gamepiece = new Gamepiece(this, index, type);
		gamepieces.add(gamepiece);
		return gamepiece;
	}

	/**
	 * Convenience overload for creating a typed gamepiece with default physical parameters.
	 *
	 * @param type the gamepiece type to create
	 * @return created gamepiece
	 */
	public Gamepiece createGamepiece(GamePieceType type) {
		return createGamepiece(type, 0.12, 0.27, 0.45);
	}

	/**
	 * Convenience overload for creating a named gamepiece type with default physical parameters.
	 *
	 * @param typeName human readable type name (e.g. "generic_sphere", "fuel_rebuilt_2026")
	 * @return created gamepiece
	 */
	public Gamepiece createGamepiece(String typeName) {
		return createGamepiece(typeName, 0.12, 0.27, 0.45);
	}

	/**
	 * Creates a new named gamepiece type (useful for season definitions or CAD-imported types).
	 *
	 * @param typeName human readable type name (e.g. "generic_sphere", "fuel_rebuilt_2026")
	 * @param radiusMeters sphere radius in meters
	 * @param massKg mass in kilograms
	 * @param restitution coefficient of restitution
	 * @return created gamepiece
	 */
	public Gamepiece createGamepiece(String typeName, double radiusMeters, double massKg, double restitution) {
		int index = JSimJNI.createGamepieceWithTypeName(worldHandle, typeName, radiusMeters, massKg, restitution);
		if (index < 0) {
			throw new JSimException(
				"Failed to create gamepiece in physics world",
				index,
				"Verify hitbox and material parameters. Radius and mass must be positive finite values.");
		}

		Gamepiece gamepiece = new Gamepiece(this, index);
		gamepieces.add(gamepiece);
		return gamepiece;
	}

	/**
	 * Returns gamepieces created through this world wrapper in insertion order.
	 *
	 * @return live list of created gamepieces
	 */
	public List<Gamepiece> gamepieces() {
		return gamepieces;
	}

	/**
	 * Registers a callback that runs after each successful physics step.
	 *
	 * @param listener callback invoked after {@link #step()} completes
	 */
	public void addStepListener(Runnable listener) {
		if (listener != null) {
			stepListeners.add(listener);
		}
	}

	/**
	 * Sets the body's world-space position in meters.
	 *
	 * @param bodyIndex native body index
	 * @param xMeters x position in meters
	 * @param yMeters y position in meters
	 * @param zMeters z position in meters
	 */
	void setBodyPosition(int bodyIndex, double xMeters, double yMeters, double zMeters) {
		int rc = JSimJNI.setBodyPosition(worldHandle, bodyIndex, xMeters, yMeters, zMeters);
		if (rc != 0) {
			throw new JSimException("Failed to set body position for bodyIndex=" + bodyIndex + " to (" + xMeters + ", " + yMeters + ", " + zMeters + ") m", rc,
				"Body index may be invalid or the world handle is corrupted. Check that bodyIndex >= 0 and the body exists in this world.");
		}
	}

	/**
	 * Sets the body's linear velocity in meters per second.
	 *
	 * @param bodyIndex native body index
	 * @param vxMetersPerSecond x velocity in meters per second
	 * @param vyMetersPerSecond y velocity in meters per second
	 * @param vzMetersPerSecond z velocity in meters per second
	 */
	void setBodyLinearVelocity(int bodyIndex, double vxMetersPerSecond, double vyMetersPerSecond,
			double vzMetersPerSecond) {
		int rc = JSimJNI.setBodyLinearVelocity(worldHandle, bodyIndex, vxMetersPerSecond,
				vyMetersPerSecond, vzMetersPerSecond);
		if (rc != 0) {
			throw new JSimException("Failed to set body linear velocity for bodyIndex=" + bodyIndex + " to (" + vxMetersPerSecond + ", " + vyMetersPerSecond + ", " + vzMetersPerSecond + ") m/s", rc,
				"Body index may be invalid or velocity values are non-finite. Check that bodyIndex >= 0 and all velocities are finite numbers.");
		}
	}

	/**
	 * Sets the body's world-space orientation.
	 *
	 * @param bodyIndex native body index
	 * @param qw quaternion w component
	 * @param qx quaternion x component
	 * @param qy quaternion y component
	 * @param qz quaternion z component
	 */
	void setBodyOrientation(int bodyIndex, double qw, double qx, double qy, double qz) {
		int rc = JSimJNI.setBodyOrientation(worldHandle, bodyIndex, qw, qx, qy, qz);
		if (rc != 0) {
			throw new JSimException("Failed to set body orientation for bodyIndex=" + bodyIndex,
				rc,
				"Body index may be invalid or orientation values are non-finite. Check that bodyIndex >= 0 and all quaternion components are finite numbers.");
		}
	}

	/**
	 * Enables or disables gravity for the given body.
	 *
	 * @param bodyIndex native body index
	 * @param enabled true to enable gravity
	 */
	void setBodyGravityEnabled(int bodyIndex, boolean enabled) {
		int rc = JSimJNI.setBodyGravityEnabled(worldHandle, bodyIndex, enabled);
		if (rc != 0) {
			throw new JSimException("Failed to set body gravity enabled=" + enabled + " for bodyIndex=" + bodyIndex, rc,
				"Body index may be invalid or does not exist. Check that bodyIndex >= 0 and refers to an existing body in this world.");
		}
	}

	/**
	 * Sets the body's collision filter for broad-phase collision detection.
	 * Bodies on the same collision layer will be filtered based on their respective
	 * collision masks, allowing fine-grained control over which bodies can collide.
	 *
	 * @param bodyIndex native body index
	 * @param collisionLayerBits collision layer bits (which group this body belongs to)
	 * @param collisionMaskBits collision mask bits (which groups this body will collide with)
	 */
	void setBodyCollisionFilter(int bodyIndex, int collisionLayerBits, int collisionMaskBits) {
		int rc = JSimJNI.setBodyCollisionFilter(worldHandle, bodyIndex, collisionLayerBits,
				collisionMaskBits);
		if (rc != 0) {
			throw new JSimException("Failed to set body collision filter for bodyIndex=" + bodyIndex + " with layerBits=" + collisionLayerBits + ", maskBits=" + collisionMaskBits, rc,
				"Body index may be invalid or does not exist. Check that bodyIndex >= 0 and refers to an existing body in this world.");
		}
	}

	/**
	 * Defines an approximate spherical collision/body shape for aerodynamic drag modeling.
	 * This sets both the collision geometry and aerodynamic drag calculations for the body.
	 *
	 * @param bodyIndex native body index
	 * @param radiusMeters sphere radius in meters
	 * @param dragCoefficient drag coefficient for aerodynamic calculations
	 */
	void setBodyAerodynamicSphere(int bodyIndex, double radiusMeters, double dragCoefficient) {
		int rc = JSimJNI.setBodyAerodynamicSphere(worldHandle, bodyIndex, radiusMeters,
				dragCoefficient);
		if (rc != 0) {
			throw new JSimException("Failed to set body aerodynamic sphere for bodyIndex=" + bodyIndex + " with radius=" + radiusMeters + " m, dragCoefficient=" + dragCoefficient, rc,
				"Body index may be invalid, or radius/drag coefficient are invalid. Radius must be positive and finite; drag coefficient must be non-negative and finite.");
		}
	}

	/**
	 * Defines an approximate box collision/body shape for aerodynamic drag modeling.
	 * This sets both the collision geometry and aerodynamic drag calculations for the body.
	 *
	 * @param bodyIndex native body index
	 * @param xMeters box x dimension in meters
	 * @param yMeters box y dimension in meters
	 * @param zMeters box z dimension in meters
	 * @param dragCoefficient drag coefficient for aerodynamic calculations
	 */
	void setBodyAerodynamicBox(int bodyIndex, double xMeters, double yMeters, double zMeters,
			double dragCoefficient) {
		int rc = JSimJNI.setBodyAerodynamicBox(worldHandle, bodyIndex, xMeters, yMeters,
				zMeters, dragCoefficient);
		if (rc != 0) {
			throw new JSimException("Failed to set body aerodynamic box for bodyIndex=" + bodyIndex + " with dimensions (" + xMeters + ", " + yMeters + ", " + zMeters + ") m, dragCoefficient=" + dragCoefficient, rc,
				"Body index may be invalid, or dimensions/drag coefficient are invalid. All dimensions must be positive and finite; drag coefficient must be non-negative and finite.");
		}
	}

	/**
	 * Sets the gamepiece's world-space position in meters.
	 *
	 * @param gamepieceIndex native gamepiece index
	 * @param xMeters x position in meters
	 * @param yMeters y position in meters
	 * @param zMeters z position in meters
	 */
	public void setGamepiecePosition(int gamepieceIndex, double xMeters, double yMeters, double zMeters) {
		int rc = JSimJNI.setGamepiecePosition(worldHandle, gamepieceIndex, xMeters, yMeters, zMeters);
		if (rc != 0) {
			throw new JSimException("Failed to set gamepiece position", rc,
				"Verify the gamepiece index and that the native world is valid; check native logs for details.");
		}
	}

	/**
	 * Requests pickup of a gamepiece into a carrier.
	 *
	 * @param gamepieceIndex native gamepiece index
	 * @param intakeX intake world x position
	 * @param intakeY intake world y position
	 * @param intakeZ intake world z position
	 * @param captureRadius pickup radius in meters
	 * @param carryOffsetX carry offset x in meters
	 * @param carryOffsetY carry offset y in meters
	 * @param carryOffsetZ carry offset z in meters
	 * @return zero on success
	 */
	public int pickGamepiece(int gamepieceIndex, double intakeX, double intakeY, double intakeZ,
				double captureRadius, double carryOffsetX, double carryOffsetY, double carryOffsetZ) {
		int rc = JSimJNI.pickGamepiece(worldHandle, gamepieceIndex,
				intakeX, intakeY, intakeZ, captureRadius,
				carryOffsetX, carryOffsetY, carryOffsetZ);
		return rc;
	}

	/**
	 * Places a gamepiece at the given world position and marks it grounded.
	 *
	 * @param gamepieceIndex native gamepiece index
	 * @param xMeters x position in meters
	 * @param yMeters y position in meters
	 * @param zMeters z position in meters
	 */
	public void placeGamepiece(int gamepieceIndex, double xMeters, double yMeters, double zMeters) {
		int rc = JSimJNI.placeGamepiece(worldHandle, gamepieceIndex, xMeters, yMeters, zMeters);
		if (rc != 0) {
			throw new JSimException("Failed to place gamepiece", rc,
				"Verify the gamepiece index and that the native world is valid; check native logs for details.");
		}
	}

	/**
	 * Outtakes a gamepiece from a pose with a linear velocity.
	 *
	 * @param gamepieceIndex native gamepiece index
	 * @param px launch x position in meters
	 * @param py launch y position in meters
	 * @param pz launch z position in meters
	 * @param vx launch x velocity in meters per second
	 * @param vy launch y velocity in meters per second
	 * @param vz launch z velocity in meters per second
	 */
	public void outtakeGamepiece(int gamepieceIndex, double px, double py, double pz,
					double vx, double vy, double vz) {
		int rc = JSimJNI.outtakeGamepiece(worldHandle, gamepieceIndex, px, py, pz, vx, vy, vz);
		if (rc != 0) {
			throw new JSimException("Failed to outtake gamepiece", rc,
				"Verify the gamepiece index and parameters; check native logs for details.");
		}
	}

	// Deprecated ball setters removed. Use gamepiece setters above.

	/**
	 * Sets the gamepiece's world-space linear velocity in meters per second.
	 *
	 * @param gamepieceIndex native gamepiece index
	 * @param vxMetersPerSecond x velocity in meters per second
	 * @param vyMetersPerSecond y velocity in meters per second
	 * @param vzMetersPerSecond z velocity in meters per second
	 */
	public void setGamepieceLinearVelocity(int gamepieceIndex, double vxMetersPerSecond,
			double vyMetersPerSecond, double vzMetersPerSecond) {
		int rc = JSimJNI.setGamepieceLinearVelocity(worldHandle, gamepieceIndex, vxMetersPerSecond,
				vyMetersPerSecond, vzMetersPerSecond);
		if (rc != 0) {
			throw new JSimException("Failed to set gamepiece linear velocity", rc,
				"Verify the gamepiece index and velocity parameters; check native logs for details.");
		}
	}

	// Deprecated ball velocity setters removed. Use gamepiece setters above.

	/**
	 * Gets the world position for the given body.
	 *
	 * @param bodyIndex native body index
	 * @return body position
	 */
	public Pose3d getBodyPosition(int bodyIndex) {
		double[] values = new double[3];
		int rc = JSimJNI.getBodyPosition(worldHandle, bodyIndex, values);
		if (rc != 0) {
			throw new JSimException("Failed to get body position", rc,
				"Verify the body index and that the world is initialized; check native logs for details.");
		}
		return new Pose3d(values[0], values[1], values[2], Rotation3d.kZero);
	}

	/**
	 * Gets the world linear velocity for the given body.
	 *
	 * @param bodyIndex native body index
	 * @return body linear velocity
	 */
	public LinearVelocity3d getBodyLinearVelocity(int bodyIndex) {
		double[] values = new double[3];
		int rc = JSimJNI.getBodyLinearVelocity(worldHandle, bodyIndex, values);
		if (rc != 0) {
			throw new JSimException("Failed to get body linear velocity", rc,
				"Verify the body index and that the world is initialized; check native logs for details.");
		}
		return new LinearVelocity3d(values[0], values[1], values[2]);
	}

	/**
	 * Gets the world orientation for the given body.
	 *
	 * @param bodyIndex native body index
	 * @return body orientation
	 */
	public Rotation3d getBodyOrientation(int bodyIndex) {
		double[] values = new double[4];
		int rc = JSimJNI.getBodyOrientation(worldHandle, bodyIndex, values);
		if (rc != 0) {
			throw new JSimException("Failed to get body orientation", rc,
				"Verify the body index and that the world is initialized; check native logs for details.");
		}
		return new Rotation3d(new Quaternion(values[0], values[1], values[2], values[3]));
	}

	/**
	 * Exports full body state blocks.
	 *
	 * <p>Layout per body is: [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz].
	 *
	 * @param outState13 destination array sized for N*13 entries
	 * @return number of body blocks written
	 */
	public int getBodyState13Array(double[] outState13) {
		int rc = JSimJNI.getBodyState13Array(worldHandle, outState13);
		if (rc < 0) {
			throw new JSimException("Failed to get body state array", rc,
				"Verify the destination array length and check native logs for details.");
		}
		return rc;
	}

	/**
	 * Gets the world position for the given gamepiece.
	 *
	 * @param gamepieceIndex native gamepiece index
	 * @return gamepiece position as a Pose3d with zero rotation
	 */
	public Pose3d getGamepiecePosition(int gamepieceIndex) {
		double[] values = new double[3];
		int rc = JSimJNI.getGamepiecePosition(worldHandle, gamepieceIndex, values);
		if (rc != 0) {
			throw new JSimException("Failed to get gamepiece position", rc,
				"Verify the gamepiece index and that the world is initialized; check native logs for details.");
		}
		return new Pose3d(values[0], values[1], values[2], Rotation3d.kZero);
	}

	/**
	 * Returns the registered type name for the given gamepiece.
	 *
	 * @param gamepieceIndex native gamepiece index
	 * @return human-readable type name or null if none
	 */
	public String getGamepieceTypeName(int gamepieceIndex) {
		return JSimJNI.getGamepieceTypeName(worldHandle, gamepieceIndex);
	}

	// Deprecated ball accessors removed. Use `getGamepiecePosition` instead.

	/**
	 * Gets the world linear velocity for the given gamepiece.
	 *
	 * @param gamepieceIndex native gamepiece index
	 * @return gamepiece linear velocity as a LinearVelocity3d
	 */
	public LinearVelocity3d getGamepieceLinearVelocity(int gamepieceIndex) {
		double[] values = new double[3];
		int rc = JSimJNI.getGamepieceLinearVelocity(worldHandle, gamepieceIndex, values);
		if (rc != 0) {
			throw new JSimException(
				"Failed to get gamepiece linear velocity for gamepieceIndex=" + gamepieceIndex,
				rc,
				"Gamepiece index may be invalid or does not exist.");
		}
		return new LinearVelocity3d(values[0], values[1], values[2]);
	}
	// Deprecated ball accessors removed. Use `getGamepieceLinearVelocity` instead.
	/**
	 * Advances the simulation by one step.
	 */
	public void step() {
		step(1);
	}

	/**
	 * Advances the simulation by the requested number of steps.
	 *
	 * @param steps number of steps to advance
	 */
	public void step(int steps) {
		int rc = JSimJNI.stepWorld(worldHandle, steps);
		if (rc != 0) {
			throw new JSimException("Failed to step world by " + steps + " step(s)", rc,
				"World handle may be corrupted or previously destroyed. Ensure the PhysicsWorld has not been closed and the world handle is valid.");
		}
		for (Runnable listener : stepListeners) {
			listener.run();
		}
	}

	/**
	 * Applies a gravity vector to the world.
	 *
	 * @param gravity gravity vector in meters per second squared
	 */
	public void setGravity(Translation3d gravity) {
		setGravity(gravity.getX(), gravity.getY(), gravity.getZ());
	}

	/**
	 * Applies gravity components to the world.
	 *
	 * @param gxMetersPerSecondSquared x gravity in meters per second squared
	 * @param gyMetersPerSecondSquared y gravity in meters per second squared
	 * @param gzMetersPerSecondSquared z gravity in meters per second squared
	 */
	public void setGravity(double gxMetersPerSecondSquared, double gyMetersPerSecondSquared,
			double gzMetersPerSecondSquared) {
		int rc = JSimJNI.setWorldGravity(worldHandle, gxMetersPerSecondSquared,
				gyMetersPerSecondSquared, gzMetersPerSecondSquared);
		if (rc != 0) {
			throw new JSimException("Failed to set world gravity to (" + gxMetersPerSecondSquared + ", " + gyMetersPerSecondSquared + ", " + gzMetersPerSecondSquared + ") m/s^2", rc,
				"Gravity values must be finite numbers. Check that all gravity components are valid floating point values.");
		}
	}

	@Override
	public void close() {
		if (worldHandle != 0) {
			JSimJNI.destroyWorld(worldHandle);
			worldHandle = 0;
		}
	}
}
