package jsim.simulation.drivesims;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
import jsim.api.SimBody;

/**
 * Kinematic WPILib hook that applies robot-relative chassis speeds to a simulated body.
 */
public final class DriveTrainSimulation {
    private final SimBody body;
    private final Supplier<ChassisSpeeds> robotRelativeSpeeds;
    private final Supplier<Pose2d> poseSupplier;

    public DriveTrainSimulation(
            SimBody body, Supplier<ChassisSpeeds> robotRelativeSpeeds, Supplier<Pose2d> poseSupplier) {
        this.body = body;
        this.robotRelativeSpeeds = robotRelativeSpeeds;
        this.poseSupplier = poseSupplier;
    }

    /** Apply the current supplier outputs to body linear and angular velocity. */
    public void apply() {
        ChassisSpeeds speeds = robotRelativeSpeeds.get();
        Rotation2d heading = poseSupplier.get().getRotation();
        double cosH = heading.getCos();
        double sinH = heading.getSin();
        body.setLinearVelocity(
                cosH * speeds.vxMetersPerSecond - sinH * speeds.vyMetersPerSecond,
                sinH * speeds.vxMetersPerSecond + cosH * speeds.vyMetersPerSecond,
                0.0);
        body.setAngularVelocity(0.0, 0.0, speeds.omegaRadiansPerSecond);
    }

    public SimBody getBody() {
        return body;
    }
}
