package jsim.field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import jsim.api.SimBody;

/**
 * A dynamic game piece backed by a {@link SimBody}, with a named variant that
 * maps to an AdvantageScope gamepiece model (e.g. {@code "Note"}, {@code "Coral"}).
 *
 * <p>Obtain instances from {@link FieldSimulator#spawnGamePiece}. All physics state
 * (pose, velocity, mass, collider) is managed through the underlying {@link SimBody}.
 */
public final class GamePiece {
    private final SimBody body;
    private final String variant;
    private boolean held;

    GamePiece(SimBody body, String variant) {
        this.body    = body;
        this.variant = variant;
    }

    /**
     * AdvantageScope gamepiece variant name. Must match the key used in the
     * AdvantageScope asset configuration (e.g. {@code "Note"} for 2024 Crescendo,
     * {@code "Coral"} for 2025 Reefscape).
     *
     * @return the variant name
     */
    public String getVariant() { return variant; }

    /**
     * The underlying physics body. Use this for direct velocity/force manipulation
     * or to attach force generators via {@link jsim.api.SimWorld#addForceGenerator}.
     *
     * @return the backing SimBody
     */
    public SimBody getBody() { return body; }

    /** @return current world-frame pose */
    public Pose3d getPose() { return body.getPose(); }

    /** @return current linear velocity in world frame (m/s) */
    public Translation3d getLinearVelocity() { return body.getLinearVelocity(); }

    /**
     * Whether this game piece is currently attached to a {@link GamePieceGripper}.
     * While held, the piece follows the robot's intake point and does not respond
     * to gravity or collisions independently.
     *
     * @return {@code true} if held by a gripper
     */
    public boolean isHeld() { return held; }

    void setHeld(boolean held) { this.held = held; }
}
