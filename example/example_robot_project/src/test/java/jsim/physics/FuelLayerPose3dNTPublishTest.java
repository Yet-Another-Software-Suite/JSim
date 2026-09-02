package jsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import jsim.physics.layers.fields.FuelLayer;
import org.dyn4j.world.World;
import org.junit.jupiter.api.Test;

class FuelLayerPose3dNTPublishTest {

  @Test
  void publishesCurrentFuelPose3dListToNetworkTables() {
    var table = NetworkTableInstance.getDefault().getTable("Mechanisms").getSubTable("fuel_test");
    var topic = table.getStructArrayTopic("poses", Pose3d.struct);
    StructArraySubscriber<Pose3d> subscriber = topic.subscribe(new Pose3d[0]);

    FuelLayer layer = new FuelLayer(new World<>(), new Translation2d(0.0, 0.0), 1, 2);
    layer.process(new Pose2d(0.0, 0.0, Rotation2d.kZero), new ChassisSpeeds(), new Translation2d(0.5, 0.5), 0.02);

    NetworkTableInstance.getDefault().flush();
    Pose3d[] poses = subscriber.get();

    assertEquals(2, poses.length, "Fuel grid should publish one Pose3d per spawned fuel piece");
    assertEquals(0.0, poses[0].getX(), 1e-9);
    assertEquals(0.0, poses[0].getY(), 1e-9);
    assertEquals(0.0, poses[0].getZ(), 1e-9);
  }
}
