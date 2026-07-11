package jsim.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.Predicate;
import jsim.field.FieldSimulator;
import jsim.field.GamePiece;

/**
 * Projectile lifecycle wrapper around a dynamic game piece.
 */
public final class GamePieceProjectile {
    private final FieldSimulator fieldSimulator;
    private final GamePiece gamePiece;
    private final Predicate<Pose3d> completionPredicate;
    private final double maxFlightSeconds;
    private double elapsedSeconds;
    private boolean completed;

    public GamePieceProjectile(
            FieldSimulator fieldSimulator,
            GamePiece gamePiece,
            Predicate<Pose3d> completionPredicate,
            double maxFlightSeconds) {
        this.fieldSimulator = fieldSimulator;
        this.gamePiece = gamePiece;
        this.completionPredicate = completionPredicate;
        this.maxFlightSeconds = maxFlightSeconds;
    }

    /** Advances projectile lifecycle and despawns on completion. */
    public void update(double dtSeconds) {
        if (completed) return;
        elapsedSeconds += dtSeconds;
        Pose3d pose = gamePiece.getPose();
        if (pose.getZ() < -0.05 || elapsedSeconds >= maxFlightSeconds || completionPredicate.test(pose)) {
            fieldSimulator.removeGamePiece(gamePiece);
            completed = true;
        }
    }

    public boolean isCompleted() {
        return completed;
    }

    public GamePiece getGamePiece() {
        return gamePiece;
    }
}
