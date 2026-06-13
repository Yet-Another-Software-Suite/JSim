// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.nt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import jsim.api.RobotID;
import jsim.api.SimRobot;
import jsim.api.StateManager;
import java.util.HashMap;
import java.util.Map;

/**
 * Publishes JSim robot poses to NetworkTables in an AdvantageScope-friendly format.
 *
 * <p>Publishes:
 * <ul>
 *   <li>Robot id labels: {@code /JSim/RobotPose/robotIds}</li>
 *   <li>Robot poses as Pose3d array: {@code /JSim/RobotPose/robotPoses}</li>
 *   <li>Robot poses as flat XYtheta arrays: {@code /JSim/RobotPose/robotPose3Flat}</li>
 *   <li>Robot poses as Pose2d array to field2d (AdvantageScope): {@code /field2d/JSim}</li>
 * </ul>
 *
 * <p>Usage example:
 * <pre>{@code
 * RobotPosePublisher publisher = new RobotPosePublisher();
 * // Publish all registered SimRobots after each physics step:
 * world.addStepListener(publisher::publishFrame);
 * // Published topics: /JSim/RobotPose/robotPoses, /field2d/JSim
 *
 * publisher.close();
 * }</pre>
 */
public class RobotPosePublisher implements AutoCloseable {
  private static final String DEFAULT_BASE_TOPIC = "JSim/RobotPose";
  private static final String FIELD2D_TOPIC = "field2d";

  private final StructArrayPublisher<Pose3d> robotPosesPublisher;
  private final StringArrayPublisher robotIdsPublisher;
  private final DoubleArrayPublisher robotPoseFlatPublisher;
  private final NetworkTableInstance ntInstance;
  private final Map<RobotID, StructPublisher<Pose2d>> field2dPublishers;
  private final StructArrayPublisher<Pose2d> field2dArrayPublisher;

  /**
   * Creates a new robot pose publisher under {@code /JSim/RobotPose}.
   */
  public RobotPosePublisher() {
    this(NetworkTableInstance.getDefault(), DEFAULT_BASE_TOPIC);
  }

  /**
   * Creates a new robot pose publisher.
   *
   * @param ntInstance network table instance
   * @param baseTopic base topic path (e.g. {@code /JSim/RobotPose})
   */
  public RobotPosePublisher(NetworkTableInstance ntInstance, String baseTopic) {
    this.ntInstance = ntInstance;
    this.field2dPublishers = new HashMap<>();

    NetworkTable table = ntInstance.getTable(baseTopic);
    this.robotPosesPublisher = table.getStructArrayTopic("robotPoses", Pose3d.struct).publish();
    this.robotIdsPublisher = table.getStringArrayTopic("robotIds").publish();
    this.robotPoseFlatPublisher = table.getDoubleArrayTopic("robotPose3Flat").publish();

    // Initialize field2d array publisher for all robots at once (AdvantageScope compatible)
    NetworkTable field2dTable = ntInstance.getTable(FIELD2D_TOPIC);
    this.field2dArrayPublisher = field2dTable.getStructArrayTopic("JSim", Pose2d.struct).publish();
  }

  /**
   * Publishes the current robot poses and names for all registered JSim robots.
   *
   * @return the number of robots published
   */
  public int publishFrame() {
    var robots = StateManager.getInstance().getRobots();
    int count = robots.size();
    if (count == 0) {
      robotIdsPublisher.set(new String[0]);
      robotPosesPublisher.set(new Pose3d[0]);
      robotPoseFlatPublisher.set(new double[0]);
      field2dArrayPublisher.set(new Pose2d[0]);
      return 0;
    }

    Pose3d[] poses = new Pose3d[count];
    Pose2d[] field2dPoses = new Pose2d[count];
    String[] ids = new String[count];
    double[] flat = new double[count * 3];

    int index = 0;
    for (var entry : robots.entrySet()) {
      RobotID robotID = entry.getKey();
      SimRobot robot = entry.getValue();
      Pose2d pose2d = robot.getPose();
      ids[index] = robotID.name();

      poses[index] = new Pose3d(
          new Translation3d(pose2d.getX(), pose2d.getY(), 0.0),
          new Rotation3d(0.0, 0.0, pose2d.getRotation().getRadians()));

      field2dPoses[index] = pose2d;

      int base = index * 3;
      flat[base] = pose2d.getX();
      flat[base + 1] = pose2d.getY();
      flat[base + 2] = pose2d.getRotation().getRadians();

      index++;
    }

    robotIdsPublisher.set(ids);
    robotPosesPublisher.set(poses);
    robotPoseFlatPublisher.set(flat);

    // Publish all robot poses to field2d for AdvantageScope visualization as Pose2d array
    field2dArrayPublisher.set(field2dPoses);

    return count;
  }

  @Override
  public void close() {
    robotIdsPublisher.close();
    robotPosesPublisher.close();
    robotPoseFlatPublisher.close();
    field2dArrayPublisher.close();
    for (StructPublisher<Pose2d> publisher : field2dPublishers.values()) {
      publisher.close();
    }
  }
}
