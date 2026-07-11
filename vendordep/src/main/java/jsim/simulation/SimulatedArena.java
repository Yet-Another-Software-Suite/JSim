package jsim.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;
import java.util.function.Supplier;
import jsim.api.SimBody;
import jsim.api.SimBodyBuilder;
import jsim.field.FieldLoader;
import jsim.field.FieldSimulator;
import jsim.field.GamePiece;
import jsim.field.GamePieceGripper;
import jsim.simulation.drivesims.DriveTrainSimulation;
import jsim.simulation.drivesims.OpponentSimulation;
import jsim.simulation.motorsims.SimulatedBattery;

/**
 * Stable top-level arena lifecycle API that orchestrates robots, opponents, and projectiles.
 */
public final class SimulatedArena {
    private final SeasonConfig seasonConfig;
    private final FieldSimulator fieldSimulator;
    private final SimulatedBattery battery;
    private final List<DriveTrainSimulation> driveTrainSimulations = new ArrayList<>();
    private final List<OpponentSimulation> opponentSimulations = new ArrayList<>();
    private final List<GamePieceProjectile> projectiles = new ArrayList<>();

    public SimulatedArena(SeasonConfig seasonConfig) {
        this.seasonConfig = seasonConfig;
        this.fieldSimulator =
                seasonConfig.fieldConstantsClass() == null
                        ? FieldLoader.fromResource(seasonConfig.fieldResource())
                        : FieldLoader.fromFieldConstantsClass(seasonConfig.fieldConstantsClass(), seasonConfig.id());
        this.battery = new SimulatedBattery();
    }

    public static SimulatedArena fromSeason(String keyOrAlias) {
        return new SimulatedArena(SeasonRegistry.resolve(keyOrAlias));
    }

    public SimBody addRobot(SimBodyBuilder builder) {
        return fieldSimulator.addRobot(builder);
    }

    public DriveTrainSimulation registerDriveTrain(
            SimBody robotBody, Supplier<ChassisSpeeds> robotRelativeSpeeds, Supplier<Pose2d> poseSupplier) {
        DriveTrainSimulation sim = new DriveTrainSimulation(robotBody, robotRelativeSpeeds, poseSupplier);
        driveTrainSimulations.add(sim);
        return sim;
    }

    public OpponentSimulation registerOpponent(
            SimBody opponentBody, Supplier<ChassisSpeeds> robotRelativeSpeeds, Supplier<Pose2d> poseSupplier) {
        OpponentSimulation sim = new OpponentSimulation(opponentBody, robotRelativeSpeeds, poseSupplier);
        opponentSimulations.add(sim);
        return sim;
    }

    public IntakeSimulation createIntake(
            SimBody robotBody, Translation3d intakeOffset, double intakeRadiusMeters) {
        GamePieceGripper gripper = fieldSimulator.createGripper(robotBody, intakeOffset);
        return new IntakeSimulation(fieldSimulator, gripper, intakeOffset, intakeRadiusMeters);
    }

    public GamePiece spawnGamePiece(String variant, SimBodyBuilder builder) {
        return fieldSimulator.spawnGamePiece(variant, builder);
    }

    public GamePieceProjectile launchProjectile(
            String variant,
            SimBodyBuilder builder,
            Predicate<edu.wpi.first.math.geometry.Pose3d> completionPredicate,
            double maxFlightSeconds) {
        GamePiece piece = spawnGamePiece(variant, builder);
        GamePieceProjectile projectile =
                new GamePieceProjectile(fieldSimulator, piece, completionPredicate, maxFlightSeconds);
        projectiles.add(projectile);
        return projectile;
    }

    /** One full arena tick: drivetrain hooks, world step, projectile lifecycle, battery publish. */
    public void simulationPeriodic() {
        for (DriveTrainSimulation sim : driveTrainSimulations) {
            sim.apply();
        }
        for (OpponentSimulation sim : opponentSimulations) {
            sim.apply();
        }

        fieldSimulator.step();

        double dt = fieldSimulator.getWorld().getTimestep();
        for (GamePieceProjectile projectile : projectiles) {
            projectile.update(dt);
        }
        projectiles.removeIf(GamePieceProjectile::isCompleted);
        battery.publishToRoboRio();
    }

    public FieldSimulator getFieldSimulator() {
        return fieldSimulator;
    }

    public SimulatedBattery getBattery() {
        return battery;
    }

    public SeasonConfig getSeasonConfig() {
        return seasonConfig;
    }
}
