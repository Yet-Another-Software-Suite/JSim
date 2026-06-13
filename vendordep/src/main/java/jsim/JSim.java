// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim;

import edu.wpi.first.math.geometry.Translation2d;
import jsim.api.RobotID;
import jsim.api.SimRobot;
import jsim.api.StateManager;
import jsim.nt.RobotPosePublisher;
import jsim.nt.WorldPosePublisher;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import jsim.field.FieldDefinitionCatalog;
import jsim.field.FieldConfig;
import jsim.api.GamePieceType;
import jsim.Gamepiece;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

/**
 * Public entry point for common JSim accessors.
 *
 * <p>Usage example:
 * <pre>{@code
 * // Wire up simulation in robotInit():
 * PhysicsWorld world = JSim.createPhysicsWorld(0.02, false);
 * StateManager.getInstance().setPhysicsWorld(world);
 * JSim.initializeField(2026);
 * SimRobot robot = JSim.createRobot(RobotID.BLUE_1, new Translation2d[]{
 *     new Translation2d( 0.35,  0.35),
 *     new Translation2d( 0.35, -0.35),
 *     new Translation2d(-0.35, -0.35),
 *     new Translation2d(-0.35,  0.35)
 * });
 *
 * // Each simulation tick (simulationPeriodic):
 * robot.setChassisSpeeds(new ChassisSpeeds(1.0, 0, 0));
 * StateManager.getInstance().stepPhysics();
 * Pose2d pose = robot.getPose();
 * }</pre>
 */
public final class JSim {
  private static PhysicsWorld physicsWorld;
  private static WorldPosePublisher defaultWorldPublisher;
  private static RobotPosePublisher defaultRobotPosePublisher;

  private JSim() {}

  private static final ObjectMapper MAPPER = new ObjectMapper();

  /** Current FRC season year, read from build.gradle wpilibVersion at build time */
  private static final int CURRENT_FRC_YEAR = loadFrcYear();

  private static int loadFrcYear() {
    try (InputStream is = JSim.class.getResourceAsStream("/jsim_version.properties")) {
      if (is != null) {
        Properties props = new Properties();
        props.load(is);
        String year = props.getProperty("frc.year");
        if (year != null) {
          return Integer.parseInt(year);
        }
      }
    } catch (IOException | NumberFormatException e) {
      // Fall back to 2026 if properties file is unavailable
    }
    return 2026;
  }

  /**
   * Returns the shared simulation state manager.
   *
   * @return the singleton state manager
   */
  public static StateManager getStateManager() {
    return StateManager.getInstance();
  }

  /**
   * Creates a simulated robot using the shared state manager.
   *
   * @param id the robot identifier
   * @param frameDimensions the robot frame perimeter vertices
   * @return the created simulated robot
   */
  public static SimRobot createRobot(RobotID id, Translation2d[] frameDimensions) {
    return SimRobot.createRobot(id, frameDimensions);
  }

  /**
   * Returns the simulated robot registered for the given driver-station id.
   *
   * @param id the robot identifier
   * @return the registered simulated robot, or {@code null} if none exists
   */
  public static SimRobot getRobot(RobotID id) {
    return StateManager.getInstance().getRobot(id);
  }

  /**
   * Creates a new physics world.
   *
   * @param fixedDtSeconds fixed simulation timestep in seconds
   * @param enableGravity whether gravity is enabled for the world
   * @return the created physics world
   */
  public static PhysicsWorld createPhysicsWorld(double fixedDtSeconds, boolean enableGravity) {
    if (physicsWorld == null) {
      physicsWorld = new PhysicsWorld(fixedDtSeconds, enableGravity);
      StateManager.getInstance().setPhysicsWorld(physicsWorld);
    }

    if (defaultWorldPublisher == null) {
      try {
        // Install default telemetry publisher for world pose exports.
        defaultWorldPublisher = new WorldPosePublisher(
            physicsWorld.getNativeHandle(), physicsWorld.getMaxBodies());
        physicsWorld.addStepListener(() -> defaultWorldPublisher.publishFrame());
      } catch (Throwable t) {
        // Swallow errors to avoid breaking world creation when NT isn't available.
      }
    }

    if (defaultRobotPosePublisher == null) {
      try {
        defaultRobotPosePublisher = new RobotPosePublisher();
        physicsWorld.addStepListener(() -> defaultRobotPosePublisher.publishFrame());
      } catch (Throwable t) {
        // Swallow errors to avoid breaking world creation when NT isn't available.
      }
    }

    return physicsWorld;
  }

  /**
   * Returns the shared physics world, if one has been created.
   *
   * @return the shared physics world, or {@code null} if none exists yet
   */
  public static PhysicsWorld getPhysicsWorld() {
    return physicsWorld;
  }

  /**
   * Loads a built-in field definition by year and initializes the simulation field state.
   * This is part of the JSim public API so examples and user code call into the vendordep.
   *
   * @param year season year (e.g. 2026)
   */
  public static void initializeField(int year) {
    JsonNode node = FieldDefinitionCatalog.loadFieldNode(year);
    try {
      FieldConfig cfg = MAPPER.treeToValue(node, FieldConfig.class);
      StateManager.getInstance().initializeField(cfg);
      // If a physics world has been created, spawn a centered 6x30 fuel grid.
      try {
        if (physicsWorld != null && year == CURRENT_FRC_YEAR) {
          double fieldLength = node.path("field_dimensions").path("length").asDouble(16.541);
          double fieldWidth = node.path("field_dimensions").path("width").asDouble(8.069);
          double centerX = fieldLength / 2.0;
          double centerY = fieldWidth / 2.0;

          final int cols = 6;
          final int rows = 30;
          // Use 2026 fuel diameter from presets ~0.15m -> radius ~0.075m
          final double diameter = 0.15;
          final double spacing = diameter * 1.05; // slight gap

          double totalWidth = cols * spacing;
          double totalHeight = rows * spacing;
          double startX = centerX - (totalWidth / 2.0) + (spacing / 2.0);
          double startY = centerY - (totalHeight / 2.0) + (spacing / 2.0);

          for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
              Gamepiece gamepiece = physicsWorld.createGamepiece(GamePieceType.FUEL);
              double x = startX + c * spacing;
              double y = startY + r * spacing;
              // Place slightly above ground by radius to avoid initial penetration.
              gamepiece.setPosition(new Translation3d(x, y, diameter * 0.5));
            }
          }
        }
      } catch (Throwable t) {
        // Swallow to avoid breaking field initialization when physics/world isn't ready.
      }
    } catch (Exception e) {
      throw new RuntimeException("Failed to initialize field for year " + year, e);
    }
  }

  /**
   * Silences WPILib joystick connection warnings if WPILib is present on the classpath.
   * This uses reflection so callers don't need a compile-time dependency on WPILib.
   */
  public static void silenceJoystickWarnings() {
    try {
      Class<?> ds = Class.forName("edu.wpi.first.wpilibj.DriverStation");
      java.lang.reflect.Method m = ds.getMethod("silenceJoystickConnectionWarning", boolean.class);
      m.invoke(null, true);
    } catch (ClassNotFoundException e) {
      // WPILib not available; nothing to do.
    } catch (Exception e) {
      throw new RuntimeException("Failed to silence joystick warnings", e);
    }
  }

}
