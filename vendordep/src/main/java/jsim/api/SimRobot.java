// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.api;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import jsim.LinearVelocity3d;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import jsim.PhysicsBody;
import jsim.api.StateManager;

/**
 * External interface for simulated FRC Robots interacting with JSim.
 *
 * <p>Usage example:
 * <pre>{@code
 * Translation2d[] frame = {
 *     new Translation2d( 0.35,  0.35),
 *     new Translation2d( 0.35, -0.35),
 *     new Translation2d(-0.35, -0.35),
 *     new Translation2d(-0.35,  0.35)
 * };
 * SimRobot robot = SimRobot.createRobot(RobotID.BLUE_1, frame);
 * robot.resetPose(new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90)));
 *
 * // Each tick:
 * robot.setChassisSpeeds(drive.getRobotRelativeSpeed());
 * StateManager.getInstance().stepPhysics();
 * Pose2d simPose = robot.getPose();           // physics-based pose
 * drive.resetOdometry(simPose);               // sync robot odometry
 * }</pre>
 */
public class SimRobot {
    private final RobotID robotID;
    private final FieldState<RobotState> stateManagerRef;
    private final Map<String, GamepieceZone> gamepieceZones = new HashMap<>();

    /**
     * Internal state for a simulated robot, including pose and speeds.
     */
    public static class RobotState {
        /**
         * The current field-relative pose of the robot.
         */
        public Pose2d robotPose = new Pose2d();
        /**
         * The robot frame vertices relative to the robot center.
         */
        public Translation2d[] frameVertices = new Translation2d[0];
        /**
         * The current chassis speeds of the robot.
         */
        public ChassisSpeeds speeds = new ChassisSpeeds();
    }

    /**
     * Protected constructor. Use SimRobot.createRobot().
     * @param id The RobotID for this robot.
     * @param stateRef The field state reference for this robot.
     */
    protected SimRobot(RobotID id, FieldState<RobotState> stateRef) {
        this.robotID = id;
        this.stateManagerRef = stateRef;
    }

    /**
     * Initializes a new robot via the unified StateManager tracking system.
     *
     * @param id The logical driver station id.
     * @param frameDimensions Translation2d[] for each vertex of the frame perimeter
     * @return the instantiated SimRobot handle.
     */
    public static SimRobot createRobot(RobotID id, Translation2d[] frameDimensions) {
        return StateManager.getInstance().initializeRobot(id, new Pose2d(), frameDimensions);
    }

    /**
     * Retrieves a previously created robot by driver-station id.
     *
     * @param id the robot identifier
     * @return the registered robot, or {@code null} if none exists
     */
    public static SimRobot getRobot(RobotID id) {
        return StateManager.getInstance().getRobot(id);
    }

    /**
     * Creates a gamepiece zone tied to this robot.
     *
     * @param name the zone name used for retrieval
     * @param zoneDimensions the zone polygon dimensions relative to the robot center
     * @param robotRotation the zone rotation relative to the robot
     * @return a new zone registered for simulation refreshes
     * @throws IllegalArgumentException if a zone with the same name already exists
     */
    public GamepieceZone createGamepieceZone(
            String name,
            Transform3d[] zoneDimensions,
            Rotation3d robotRotation) {
        if (name != null && gamepieceZones.containsKey(name)) {
            throw new IllegalArgumentException("Gamepiece zone already exists: " + name);
        }
        GamepieceZone zone = new GamepieceZone(this, name, zoneDimensions, robotRotation);
        if (name != null) {
            gamepieceZones.put(name, zone);
        }
        return zone;
    }

    /**
     * Retrieves a previously created gamepiece zone by name.
     *
     * @param name the zone name
     * @return the registered zone, or {@code null} if none exists
     */
    public GamepieceZone getGamepieceZone(String name) {
        return gamepieceZones.get(name);
    }

    /**
     * Returns all gamepiece zones registered on this robot.
     *
     * @return an immutable view of the named zones
     */
    public Map<String, GamepieceZone> getGamepieceZones() {
        return Collections.unmodifiableMap(gamepieceZones);
    }

    /**
     * Retrieves the current field-relative physics pose when physics is attached.
     * Falls back to the commanded pose until a physics body is registered.
     * @return the current pose of the robot.
     */
    public Pose2d getPose() {
        PhysicsBody robotBody = StateManager.getInstance().getRobotBody(robotID);
        if (robotBody != null) {
            Pose3d physicsPos = robotBody.position();
            Rotation3d physicsRot = robotBody.orientation();
            return new Pose2d(physicsPos.getX(), physicsPos.getY(), new Rotation2d(physicsRot.getZ()));
        }
        return stateManagerRef.get().robotPose;
    }

    /**
     * Retrieves the current odometry pose. This is always the commanded pose, not the physics body pose.
     * Used internally for syncing with the physics engine.
     * @return the current odometry pose of the robot.
     */
    public Pose2d getOdometryPose() {
        return stateManagerRef.get().robotPose;
    }

    /**
     * Returns the robot frame perimeter vertices used to build the robot body.
     *
     * @return cloned frame perimeter vertices
     */
    public Translation2d[] getFrameVertices() {
        Translation2d[] frameVertices = stateManagerRef.get().frameVertices;
        return frameVertices == null ? new Translation2d[0] : frameVertices.clone();
    }

    /**
     * Returns the robot's current measured chassis velocity when physics is attached.
     * Falls back to the last commanded chassis speeds until a body is registered.
     *
     * @return the current chassis velocity snapshot
     */
    public ChassisSpeeds getVelocity() {
        PhysicsBody robotBody = StateManager.getInstance().getRobotBody(robotID);
        if (robotBody != null) {
            LinearVelocity3d velocity = robotBody.linearVelocity();
            return new ChassisSpeeds(
                velocity.getVxMetersPerSecond(),
                velocity.getVyMetersPerSecond(),
                0.0);
        }

        return stateManagerRef.get().speeds;
    }

    /**
     * Hard overrides the state manager's simulation pose for this robot.
     * @param pose the new pose to set for the robot.
     */
    public void resetPose(Pose2d pose) {
        stateManagerRef.get().robotPose = pose;
    }

    /**
     * Applies commanded speeds to the physics solver.
     * @param speeds the chassis speeds to apply.
     */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        stateManagerRef.get().speeds = speeds;
    }

    /**
     * Advances the robot state by one simulation tick.
     *
     * @param dtSeconds timestep in seconds
     */
    public void update(double dtSeconds) {
        if (dtSeconds <= 0.0) {
            return;
        }

        RobotState state = stateManagerRef.get();
        if (state == null || state.robotPose == null || state.speeds == null) {
            return;
        }

        double xMeters = state.robotPose.getX() + (state.speeds.vxMetersPerSecond * dtSeconds);
        double yMeters = state.robotPose.getY() + (state.speeds.vyMetersPerSecond * dtSeconds);
        double thetaRadians =
            state.robotPose.getRotation().getRadians() + (state.speeds.omegaRadiansPerSecond * dtSeconds);
        state.robotPose = new Pose2d(xMeters, yMeters, new Rotation2d(thetaRadians));
    }

    /**
     * Gets the RobotID for this robot.
     * @return the RobotID assigned to this robot.
     */
    public RobotID getRobotID() {
        return robotID;
    }
}
