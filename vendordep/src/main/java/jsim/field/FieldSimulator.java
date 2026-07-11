package jsim.field;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import jsim.api.SimBody;
import jsim.api.SimBodyBuilder;
import jsim.api.SimWorld;
import jsim.core.SimConstants;

/**
 * High-level FRC field simulator that combines a {@link SimWorld} with pre-built
 * field geometry and game-piece management tailored to AdvantageScope integration.
 *
 * <p>The field is static — it deflects game pieces and robots but cannot itself be
 * moved. Game pieces are dynamic rigid bodies that collide with the field and each
 * other. Robots are dynamic bodies that the user drives externally (e.g. via
 * {@code SimBody.getActuator()} or {@code SimBody.setPose()}).
 *
 * <p>Typical usage:
 * <pre>{@code
 * FieldSimulator sim = new FieldSimulator();
 *
 * SimBody robot = sim.addRobot(new SimBodyBuilder("Robot")
 *     .pose(new Pose3d(2, 4, 0.05, new Rotation3d()))
 *     .mass(50)
 *     .boxCollider(0.45, 0.45, 0.3)
 *     .material(Material.CARPET)
 *     .noGravity()
 *     .fixedRotation());
 *
 * GamePieceGripper gripper = sim.createGripper(robot, new Translation3d(0.5, 0, 0.1));
 *
 * GamePiece note = sim.spawnGamePiece("Note", new SimBodyBuilder("Note0")
 *     .position(8, 4, 0.18)
 *     .mass(0.235)
 *     .sphereCollider(0.18)
 *     .material(Material.RUBBER));
 *
 * // In simulationPeriodic():
 * sim.step();
 * Pose3d[] notePoses = sim.getGamePiecePoses("Note");
 * Logger.recordOutput("FieldSimulation/NotePoses", notePoses);
 * }</pre>
 */
public final class FieldSimulator {
    private final SimWorld world;
    private SeasonFieldSpec seasonFieldSpec;
    private final List<GamePiece>        pieces   = new ArrayList<>();
    private final List<GamePieceGripper> grippers = new ArrayList<>();

    /**
     * Create a simulator with the standard FRC field (16.46 m × 8.23 m) and the
     * default 20 ms fixed timestep.
     */
    public FieldSimulator() {
        this(SimConstants.DEFAULT_DT);
    }

    /**
     * Create a simulator with the standard FRC field and a custom fixed timestep.
     *
     * @param fixedTimestepSeconds physics timestep in seconds
     */
    public FieldSimulator(double fixedTimestepSeconds) {
        this(fixedTimestepSeconds, FieldLayout.FRC_LENGTH_M, FieldLayout.FRC_WIDTH_M);
    }

    /**
     * Create a simulator with a custom field size and timestep.
     *
     * @param fixedTimestepSeconds physics timestep in seconds
     * @param fieldLengthM         field length along X (metres)
     * @param fieldWidthM          field width along Y (metres)
     */
    public FieldSimulator(double fixedTimestepSeconds, double fieldLengthM, double fieldWidthM) {
        world = new SimWorld(fixedTimestepSeconds);
        FieldLayout.addField(world, fieldLengthM, fieldWidthM);
    }

    // Body management

    /**
     * Add a robot body to the simulation.
     *
     * <p>Robots are typically configured with {@code .noGravity().fixedRotation()} and
     * driven by setting their pose or actuator forces each tick.
     *
     * @param builder configured body builder
     * @return the resulting {@link SimBody}
     */
    public SimBody addRobot(SimBodyBuilder builder) {
        return world.addBody(builder);
    }

    /**
     * Spawn a game piece in the world. The piece collides with the field (floor and walls)
     * and any robot bodies, and can be picked up by a {@link GamePieceGripper}.
     *
     * <p>The {@code variant} string must match the gamepiece key in the AdvantageScope
     * asset configuration (e.g. {@code "Note"} for 2024 Crescendo, {@code "Coral"} for
     * 2025 Reefscape).
     *
     * @param variant AdvantageScope gamepiece variant name
     * @param builder configured body builder for physics properties
     * @return the new {@link GamePiece} handle
     */
    public GamePiece spawnGamePiece(String variant, SimBodyBuilder builder) {
        SimBody body  = world.addBody(builder);
        GamePiece piece = new GamePiece(body, variant);
        pieces.add(piece);
        return piece;
    }

    /**
     * Remove a game piece from the simulation, releasing it from any gripper that
     * holds it first.
     *
     * @param piece the piece to remove
     */
    public void removeGamePiece(GamePiece piece) {
        // Detach from any holding gripper before removing the body.
        for (GamePieceGripper g : grippers) {
            if (g.getHeld() == piece) {
                g.release();
                break;
            }
        }
        world.removeBody(piece.getBody());
        pieces.remove(piece);
    }

    /**
     * Create a {@link GamePieceGripper} that mounts on {@code robot} at {@code intakeOffset}
     * (in robot-local frame). The gripper's {@link GamePieceGripper#update()} is called
     * automatically by {@link #step()}.
     *
     * @param robot        the robot body the gripper is mounted on
     * @param intakeOffset hold-point position in the robot's local frame (metres)
     * @return a new {@link GamePieceGripper}
     */
    public GamePieceGripper createGripper(SimBody robot, Translation3d intakeOffset) {
        GamePieceGripper gripper = new GamePieceGripper(robot, intakeOffset);
        grippers.add(gripper);
        return gripper;
    }

    // Stepping

    /**
     * Advance physics by the configured fixed timestep, then update all active grippers
     * so held pieces track the robot.
     *
     * <p>Call once per robot loop iteration from {@code simulationPeriodic()}.
     */
    public void step() {
        world.step();
        for (GamePieceGripper g : grippers) g.update();
    }

    // AdvantageScope output

    /**
     * Return the current {@link Pose3d} of every game piece matching {@code variant}.
     *
     * <p>Log the returned array to AdvantageScope via AdvantageKit or NT4:
     * <pre>{@code
     * Logger.recordOutput("FieldSimulation/NotePoses", sim.getGamePiecePoses("Note"));
     * }</pre>
     *
     * @param variant the variant name to filter by (case-sensitive)
     * @return array of poses, one per matching piece, in spawn order
     */
    public Pose3d[] getGamePiecePoses(String variant) {
        List<Pose3d> poses = new ArrayList<>();
        for (GamePiece p : pieces) {
            if (p.getVariant().equals(variant)) poses.add(p.getPose());
        }
        return poses.toArray(new Pose3d[0]);
    }

    /**
     * Return all game piece poses grouped by variant. Useful when multiple game
     * piece types exist on the field simultaneously.
     *
     * @return map from variant name to pose array, in spawn order within each variant
     */
    public Map<String, Pose3d[]> getAllGamePiecePoses() {
        Map<String, List<Pose3d>> grouped = new LinkedHashMap<>();
        for (GamePiece p : pieces) {
            grouped.computeIfAbsent(p.getVariant(), k -> new ArrayList<>()).add(p.getPose());
        }
        Map<String, Pose3d[]> result = new LinkedHashMap<>();
        grouped.forEach((k, v) -> result.put(k, v.toArray(new Pose3d[0])));
        return result;
    }

    /**
     * Unmodifiable view of all game pieces currently in the simulation.
     *
     * @return all game pieces in spawn order
     */
    public List<GamePiece> getGamePieces() {
        return Collections.unmodifiableList(pieces);
    }

    /**
     * Direct access to the underlying {@link SimWorld} for advanced use — custom force
     * generators, additional bodies, sensors, etc.
     *
     * @return the underlying physics world
     */
    public SimWorld getWorld() { return world; }

    public SeasonFieldSpec getSeasonFieldSpec() {
        return seasonFieldSpec;
    }

    void setSeasonFieldSpec(SeasonFieldSpec seasonFieldSpec) {
        this.seasonFieldSpec = seasonFieldSpec;
    }
}
