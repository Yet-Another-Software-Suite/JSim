package jsim.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Predicate;
import jsim.field.FieldSimulator;
import jsim.field.GamePiece;
import jsim.field.GamePieceGripper;

/**
 * Intake abstraction built on top of {@link GamePieceGripper}.
 */
public final class IntakeSimulation {
    private final FieldSimulator fieldSimulator;
    private final GamePieceGripper gripper;
    private final double intakeRadiusMeters;
    private final Translation3d intakeOffset;

    public IntakeSimulation(
            FieldSimulator fieldSimulator,
            GamePieceGripper gripper,
            Translation3d intakeOffset,
            double intakeRadiusMeters) {
        this.fieldSimulator = fieldSimulator;
        this.gripper = gripper;
        this.intakeOffset = intakeOffset;
        this.intakeRadiusMeters = intakeRadiusMeters;
    }

    /** Attempts to grab the nearest unheld game piece within radius. */
    public boolean intakeNearest(Predicate<GamePiece> filter) {
        if (gripper.isHolding()) return false;
        Pose3d robotPose = gripperRobotPose();
        Translation3d intakePoint =
                intakeOffset.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
        GamePiece closest = null;
        double best = intakeRadiusMeters;
        for (GamePiece piece : fieldSimulator.getGamePieces()) {
            if (piece.isHeld() || !filter.test(piece)) continue;
            double distance = piece.getPose().getTranslation().getDistance(intakePoint);
            if (distance < best) {
                best = distance;
                closest = piece;
            }
        }
        return closest != null && gripper.grab(closest);
    }

    public void eject(double vx, double vy, double vz) {
        gripper.eject(vx, vy, vz);
    }

    public boolean isHolding() {
        return gripper.isHolding();
    }

    public GamePiece getHeld() {
        return gripper.getHeld();
    }

    private Pose3d gripperRobotPose() {
        return gripper.getRobotBody().getPose();
    }
}
