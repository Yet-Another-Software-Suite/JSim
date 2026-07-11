package jsim.simulation.drivesims;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
import jsim.api.SimBody;

/**
 * Lightweight abstraction for opponent-robot motion driven by supplier-based commands.
 */
public final class OpponentSimulation {
    private final DriveTrainSimulation driveTrainSimulation;

    public OpponentSimulation(
            SimBody opponentBody, Supplier<ChassisSpeeds> robotRelativeSpeeds, Supplier<Pose2d> poseSupplier) {
        this.driveTrainSimulation = new DriveTrainSimulation(opponentBody, robotRelativeSpeeds, poseSupplier);
    }

    public void apply() {
        driveTrainSimulation.apply();
    }

    public SimBody getBody() {
        return driveTrainSimulation.getBody();
    }
}
