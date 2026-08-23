package jsim.physics.layers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldPublisher {
    private final Field2d field = new Field2d();

    public FieldPublisher(String name) {
        SmartDashboard.putData(name, field);
    }

    public void updateRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    public void setObjectPose(String name, Pose2d pose) {
        field.getObject(name).setPose(pose);
    }
}