package jsim.field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import jsim.api.SimBody;

/**
 * Attaches a {@link GamePiece} to a robot body's intake point so it is carried
 * rigidly in robot-local space each tick.
 *
 * <p>While a game piece is held, its pose and linear velocity are overridden each
 * tick (by {@link #update()}) to track the robot. The piece's physics body is not
 * removed from the world — it just gets corrected immediately after every physics
 * step so it always appears at the intake position.
 *
 * <p>Obtain instances from {@link FieldSimulator#createGripper}. {@link #update()}
 * is called automatically by {@link FieldSimulator#step()}, so manual calls are
 * only needed when using the gripper with a raw {@link jsim.api.SimWorld}.
 *
 * <p>Example:
 * <pre>{@code
 * GamePieceGripper gripper = sim.createGripper(robot, new Translation3d(0.45, 0, 0.1));
 *
 * // Robot drives up to a game piece and intakes it
 * gripper.grab(gamePiece);
 *
 * // ... robot drives to the speaker, then shoots ...
 * gripper.eject(0, 0, 8);   launch straight up at 8 m/s
 * }</pre>
 */
public final class GamePieceGripper {
    private final SimBody robot;
    /** Hold-point offset in the robot's local frame. */
    private final Translation3d intakeOffset;
    private GamePiece held;

    GamePieceGripper(SimBody robot, Translation3d intakeOffset) {
        this.robot        = robot;
        this.intakeOffset = intakeOffset;
    }

    /**
     * Attach a game piece to this gripper. The piece is immediately snapped to the
     * intake position and its velocity is matched to the robot's.
     *
     * <p>Returns {@code false} (no-op) if:
     * <ul>
     *   <li>this gripper is already holding a piece, or
     *   <li>{@code piece} is already held by another gripper.
     * </ul>
     *
     * @param piece the game piece to grab
     * @return {@code true} if the grab succeeded
     */
    public boolean grab(GamePiece piece) {
        if (held != null || piece.isHeld()) return false;
        held = piece;
        piece.setHeld(true);
        snapToIntake();
        return true;
    }

    /**
     * Release the held piece, giving it the robot's current linear velocity.
     * No-op if nothing is held.
     */
    public void release() {
        if (held == null) return;
        held.getBody().setLinearVelocity(robot.getLinearVelocity());
        held.getBody().setAngularVelocity(new Translation3d());
        held.setHeld(false);
        held = null;
    }

    /**
     * Eject the held piece with a custom world-frame velocity. Use this to model
     * a shooter or roller that launches the piece at a specific velocity.
     * No-op if nothing is held.
     *
     * @param vx ejection velocity X component (m/s)
     * @param vy ejection velocity Y component (m/s)
     * @param vz ejection velocity Z component (m/s)
     */
    public void eject(double vx, double vy, double vz) {
        if (held == null) return;
        held.getBody().setLinearVelocity(vx, vy, vz);
        held.getBody().setAngularVelocity(new Translation3d());
        held.setHeld(false);
        held = null;
    }

    /**
     * Teleport the held piece to the current intake position and match its velocity
     * to the robot. Called automatically by {@link FieldSimulator#step()} after
     * each physics tick.
     */
    void update() {
        if (held == null) return;
        snapToIntake();
    }

    /** @return the currently held game piece, or {@code null} if nothing is held */
    public GamePiece getHeld() { return held; }

    /** @return {@code true} if this gripper is currently holding a piece */
    public boolean isHolding() { return held != null; }

    // Internals

    private void snapToIntake() {
        held.getBody().setPose(holdPose());
        held.getBody().setLinearVelocity(robot.getLinearVelocity());
        held.getBody().setAngularVelocity(new Translation3d());
    }

    private Pose3d holdPose() {
        Pose3d robotPose = robot.getPose();
        // Rotate the local-frame intake offset into world frame, then add to robot position.
        Translation3d worldOffset = intakeOffset.rotateBy(robotPose.getRotation());
        Translation3d worldPos    = robotPose.getTranslation().plus(worldOffset);
        return new Pose3d(worldPos, robotPose.getRotation());
    }
}
