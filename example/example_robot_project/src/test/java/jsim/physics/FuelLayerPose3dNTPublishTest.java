package jsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import jsim.physics.layers.fields.Field2026;
import jsim.physics.layers.fields.FuelLayer;
import org.junit.jupiter.api.Test;

class FuelLayerPose3dNTPublishTest {

  private static final Pose2d ROBOT_POSE = new Pose2d(1.0, 1.0, Rotation2d.kZero);
  private static final Translation2d ROBOT_DIMENSIONS = new Translation2d(0.4, 0.4);

  @Test
  void publishesCurrentFuelPose3dListToNetworkTables() {
    var table = NetworkTableInstance.getDefault().getTable("Mechanisms").getSubTable("fuel_test");
    var topic = table.getStructArrayTopic("poses", Pose3d.struct);
    StructArraySubscriber<Pose3d> subscriber = topic.subscribe(new Pose3d[0]);

    FuelLayer layer = new FuelLayer(new Field2026(), "fuel_test");
    layer.clearFuel();
    layer.spawnFuel(new Translation3d(6.0, 3.0, 1.0));
    layer.spawnFuel(new Translation3d(6.5, 3.0, 1.0));

    layer.process(ROBOT_POSE, new ChassisSpeeds(), ROBOT_DIMENSIONS, 0.02);

    NetworkTableInstance.getDefault().flush();
    Pose3d[] poses = subscriber.get();

    assertEquals(2, poses.length, "Fuel layer should publish one Pose3d per live fuel piece");
    assertEquals(6.0, poses[0].getX(), 1e-9);
    assertEquals(3.0, poses[0].getY(), 1e-9);
    assertTrue(poses[0].getZ() < 1.0, "Airborne fuel should have fallen after a physics step");
  }

  @Test
  void publishesHubScoresToNetworkTables() {
    var table = NetworkTableInstance.getDefault().getTable("Mechanisms").getSubTable("fuel_score");
    var blueScore = table.getIntegerTopic("blueScore").subscribe(-1);
    var redScore = table.getIntegerTopic("redScore").subscribe(-1);

    FuelLayer layer = new FuelLayer(new Field2026(), "fuel_score");
    layer.clearFuel();
    layer.process(ROBOT_POSE, new ChassisSpeeds(), ROBOT_DIMENSIONS, 0.02);

    NetworkTableInstance.getDefault().flush();
    assertEquals(0, blueScore.get());
    assertEquals(0, redScore.get());
  }
}
