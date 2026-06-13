// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.api;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import jsim.Gamepiece;
import jsim.Vec3;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import jsim.PhysicsBody;
import jsim.PhysicsWorld;
import jsim.field.FieldConfig;
import jsim.field.FieldElement;
import java.util.Objects;

/**
 * System Rules:
 * - Single source of truth for all simulation state.
 * - Only system allowed to mutate FieldState&lt;T&gt;.
 * - External APIs are read/write proxies ONLY through defined methods.
 *
 * <p>Usage example:
 * <pre>{@code
 * // One-time setup (robotInit / simulationInit):
 * PhysicsWorld world = JSim.createPhysicsWorld(0.02, false);
 * StateManager sm = StateManager.getInstance();
 * sm.setPhysicsWorld(world);
 *
 * SimRobot robot = JSim.createRobot(RobotID.BLUE_1, frameVertices);
 * GamepieceZone intake = robot.createGamepieceZone("intake", intakeDims, Rotation3d.kZero);
 * sm.registerGamepieceZone(intake);
 * intake.intake(MetersPerSecond.of(3.0), Rotation3d.kZero);
 *
 * // Each simulationPeriodic() call:
 * robot.setChassisSpeeds(drive.getRobotRelativeSpeed());
 * sm.stepPhysics(); // advances world, dead-reckons robots, refreshes zones
 * Pose2d pose = robot.getPose();
 * }</pre>
 */
public class StateManager {
    private static final StateManager INSTANCE = new StateManager();
    private static final double DEFAULT_STEP_SECONDS = 0.02;

    private final Map<RobotID, FieldState<SimRobot.RobotState>> robotStates = new EnumMap<>(RobotID.class);
    private final Map<RobotID, SimRobot> robots = new EnumMap<>(RobotID.class);
    private final Map<RobotID, PhysicsBody> robotBodies = new EnumMap<>(RobotID.class);
    private final List<GamepieceZone> gamepieceZones = new ArrayList<>();
    private final List<FieldElement> fieldElements = new ArrayList<>();
    private PhysicsWorld physicsWorld;

    private StateManager() {}

    /**
     * Gets the singleton instance of the StateManager.
     * @return the StateManager instance.
     */
    public static StateManager getInstance() {
        return INSTANCE;
    }

    /**
     * Clears all tracked simulation state and detaches the active physics world.
     *
     * <p>Primarily used by tests to ensure deterministic, isolated state between runs.
     */
    public synchronized void reset() {
        robotStates.clear();
        robots.clear();
        robotBodies.clear();
        gamepieceZones.clear();
        fieldElements.clear();
        physicsWorld = null;
    }

    /**
     * Loads JSON field definition and builds all FieldElements and collision zones.
     *
     * @param config The field configuration to initialize.
     */
    public void initializeField(FieldConfig config) {
        Objects.requireNonNull(config, "FieldConfig must not be null");
        // Populate internal list of field elements from the parsed configuration.
        synchronized (this) {
            fieldElements.clear();
            if (config.fieldElements != null) {
                for (FieldElement e : config.fieldElements) {
                    if (e != null) {
                        fieldElements.add(e);
                    }
                }
            }
        }
    }

    /**
     * Returns an immutable view of the current field elements registered in the simulation.
     *
     * @return unmodifiable list of field elements
     */
    public synchronized List<FieldElement> getFieldElements() {
        return Collections.unmodifiableList(new ArrayList<>(fieldElements));
    }

    /**
     * Creates a robot instance, assigns starting pose, and registers it in the simulation.
     *
     * @param id The RobotID for the robot.
     * @param startingPose The initial pose of the robot.
     * @param frameVertices The vertices of the robot's frame.
     * @return the initialized SimRobot instance.
     */
    public SimRobot initializeRobot(RobotID id, Pose2d startingPose, Translation2d[] frameVertices) {
        SimRobot.RobotState internalState = new SimRobot.RobotState();
        internalState.robotPose = startingPose;
        internalState.frameVertices = frameVertices == null ? new Translation2d[0] : frameVertices.clone();

        FieldState<SimRobot.RobotState> stateRef = new FieldState<>(internalState);
        robotStates.put(id, stateRef);
        SimRobot robot = new SimRobot(id, stateRef);
        robots.put(id, robot);

        if (physicsWorld != null) {
            registerRobotBody(id, startingPose, internalState.frameVertices);
        }
        return robot;
    }

    /**
     * Returns the simulated robot registered for the given driver-station id.
     *
     * @param id the robot identifier
     * @return the registered robot, or {@code null} if none exists
     */
    public synchronized SimRobot getRobot(RobotID id) {
        return robots.get(id);
    }

    /**
     * Returns a snapshot of all registered robot poses keyed by robot id.
     *
     * @return immutable map of robot ids to poses
     */
    public Map<RobotID, Pose2d> getRobotPoses() {
        Map<RobotID, Pose2d> poses = new EnumMap<>(RobotID.class);
        for (Map.Entry<RobotID, FieldState<SimRobot.RobotState>> entry : robotStates.entrySet()) {
            poses.put(entry.getKey(), entry.getValue().get().robotPose);
        }
        return Collections.unmodifiableMap(poses);
    }

    /**
     * Returns all registered SimRobot instances by RobotID.
     *
     * @return an immutable map of robot ids to their SimRobot instances
     */
    public synchronized Map<RobotID, SimRobot> getRobots() {
        return Collections.unmodifiableMap(new EnumMap<>(robots));
    }

    /**
     * Stores the active physics world used by the simulation layer.
     *
     * @param physicsWorld the physics world to store
     */
    public synchronized void setPhysicsWorld(PhysicsWorld physicsWorld) {
        this.physicsWorld = physicsWorld;
        robotBodies.clear();

        if (physicsWorld != null) {
            for (Map.Entry<RobotID, FieldState<SimRobot.RobotState>> entry : robotStates.entrySet()) {
                RobotID id = entry.getKey();
                SimRobot.RobotState state = entry.getValue().get();
                if (state != null) {
                    registerRobotBody(id, state.robotPose, state.frameVertices);
                }
            }
        }
    }

    /**
     * Returns the active physics world used by the simulation layer.
     *
     * @return the current physics world, or {@code null} if none is registered
     */
    public synchronized PhysicsWorld getPhysicsWorld() {
        return physicsWorld;
    }

    /**
     * Associates a native physics body with a robot id.
     *
     * @param id the robot identifier
     * @param robotBody the physics body to store, or {@code null} to clear the mapping
     */
    public synchronized void setRobotBody(RobotID id, PhysicsBody robotBody) {
        if (robotBody == null) {
            robotBodies.remove(id);
        } else {
            robotBodies.put(id, robotBody);
        }
    }

    /**
     * Returns the physics body associated with the given robot id.
     *
     * @param id the robot identifier
     * @return the tracked physics body, or {@code null} if none exists
     */
    public synchronized PhysicsBody getRobotBody(RobotID id) {
        return robotBodies.get(id);
    }

    /**
     * Applies a chassis-speed command to the physics body registered for the given robot id.
     *
     * @param id the robot identifier
     * @param speeds the commanded chassis speeds
     */
    public synchronized void setPhysicsVelocity(RobotID id, ChassisSpeeds speeds) {
        PhysicsBody robotBody = robotBodies.get(id);
        if (robotBody != null) {
            robotBody.setLinearVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0.0);
        }
    }

    private void registerRobotBody(RobotID id, Pose2d pose, Translation2d[] frameVertices) {
        if (physicsWorld == null || id == null) {
            return;
        }

        PhysicsBody existingBody = robotBodies.get(id);
        if (existingBody != null) {
            return;
        }

        PhysicsBody robotBody = physicsWorld.createBody(50.0);
        robotBody.setGravityEnabled(false);

        double widthMeters = 0.7;
        double depthMeters = 0.7;
        if (frameVertices != null && frameVertices.length > 0) {
            double minX = frameVertices[0].getX();
            double maxX = frameVertices[0].getX();
            double minY = frameVertices[0].getY();
            double maxY = frameVertices[0].getY();
            for (Translation2d vertex : frameVertices) {
                if (vertex == null) {
                    continue;
                }
                minX = Math.min(minX, vertex.getX());
                maxX = Math.max(maxX, vertex.getX());
                minY = Math.min(minY, vertex.getY());
                maxY = Math.max(maxY, vertex.getY());
            }
            widthMeters = Math.max(0.05, maxX - minX);
            depthMeters = Math.max(0.05, maxY - minY);
        }

        robotBody.setCollisionBox(widthMeters, depthMeters, 0.35);
        robotBody.setPosition(new Pose3d(pose.getX(), pose.getY(), 0.175,
                new Rotation3d(0.0, 0.0, pose.getRotation().getRadians())));
        robotBody.setOrientation(new Rotation3d(0.0, 0.0, pose.getRotation().getRadians()));
        robotBody.setLinearVelocity(0.0, 0.0, 0.0);
        robotBody.setCollisionFilter(0xFFFF, 0xFFFF);
        robotBodies.put(id, robotBody);
    }

    private static Translation2d[] transformZonePolygon(Pose2d robotPose, GamepieceZone zone) {
        Transform3d[] zoneDimensions = zone.getZoneDimensions();
        Translation2d[] polygon = new Translation2d[zoneDimensions.length];
        double yaw = robotPose.getRotation().getRadians() + zone.getRobotRotation().getZ();
        double cosYaw = Math.cos(yaw);
        double sinYaw = Math.sin(yaw);

        for (int i = 0; i < zoneDimensions.length; i++) {
            Translation3d local = zoneDimensions[i].getTranslation();
            double x = robotPose.getX() + (local.getX() * cosYaw) - (local.getY() * sinYaw);
            double y = robotPose.getY() + (local.getX() * sinYaw) + (local.getY() * cosYaw);
            polygon[i] = new Translation2d(x, y);
        }

        return polygon;
    }

    private static boolean pointInPolygon(double x, double y, Translation2d[] polygon) {
        if (polygon == null || polygon.length < 3) {
            return false;
        }

        boolean inside = false;
        for (int i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
            Translation2d pi = polygon[i];
            Translation2d pj = polygon[j];
            if (pi == null || pj == null) {
                continue;
            }

            boolean intersects = ((pi.getY() > y) != (pj.getY() > y))
                    && (x < (pj.getX() - pi.getX()) * (y - pi.getY())
                        / ((pj.getY() - pi.getY()) == 0.0 ? 1e-9 : (pj.getY() - pi.getY())) + pi.getX());
            if (intersects) {
                inside = !inside;
            }
        }
        return inside;
    }

    /**
     * Registers a gamepiece zone to be refreshed on each simulation step.
     *
     * @param gamepieceZone the zone to register
     */
    public synchronized void registerGamepieceZone(GamepieceZone gamepieceZone) {
        if (gamepieceZone != null && !gamepieceZones.contains(gamepieceZone)) {
            gamepieceZones.add(gamepieceZone);
        }
    }

    private synchronized void updateGamepieceZones() {
        for (GamepieceZone gamepieceZone : gamepieceZones) {
            gamepieceZone.update();
        }
        refreshZoneGamepieceActions();
    }

    private synchronized void refreshZoneGamepieceActions() {
        // Apply zone-driven actions: when a zone is actively OUTTAKE or SHOOT,
        // convert nearby grounded gamepieces into airborne (full physics) by
        // calling the PhysicsWorld shoot API. For INTAKE, attempt pickup.
        if (physicsWorld == null) {
            return;
        }

        final double kCaptureThresholdM = 0.5; // proximity threshold (tuned)

        for (GamepieceZone zone : gamepieceZones) {
            GamepieceZone.Mode mode = zone.getMode();
            if (mode == GamepieceZone.Mode.DISABLED) {
                continue;
            }
            SimRobot owner = zone.getRobot();
            if (owner == null) {
                continue;
            }

            Pose2d robotPose = owner.getPose();
            Translation3d exitTrans = zone.getExitTranslation();
            double zoneYaw = robotPose.getRotation().getRadians() + zone.getRobotRotation().getZ();
            double cosYaw = Math.cos(zoneYaw);
            double sinYaw = Math.sin(zoneYaw);

            // world-space exit point: robot pose + rotated zone-local translation
            double wx = robotPose.getX() + (exitTrans.getX() * cosYaw) - (exitTrans.getY() * sinYaw);
            double wy = robotPose.getY() + (exitTrans.getX() * sinYaw) + (exitTrans.getY() * cosYaw);
            double wz = exitTrans.getZ();

            double exitSpeed = zone.getExitVelocity().in(MetersPerSecond);
            // Use zone rotation yaw as heading offset and Y as pitch.
            double pitch = zone.getExitRotation().getY();
            double heading = zoneYaw + zone.getExitRotation().getZ();

            Translation2d[] worldPolygon = transformZonePolygon(robotPose, zone);

            for (Gamepiece gamepiece : physicsWorld.gamepieces()) {
                Pose3d gamepiecePose = physicsWorld.getGamepiecePosition(gamepiece.gamepieceIndex());
                double dx = gamepiecePose.getTranslation().getX() - wx;
                double dy = gamepiecePose.getTranslation().getY() - wy;
                double dz = gamepiecePose.getTranslation().getZ() - wz;
                final double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
                boolean insideZone = pointInPolygon(
                        gamepiecePose.getTranslation().getX(),
                        gamepiecePose.getTranslation().getY(),
                        worldPolygon);
                if (dist > kCaptureThresholdM && !insideZone) {
                    continue;
                }

                if (mode == GamepieceZone.Mode.INTAKE) {
                    // Attempt pickup - use small capture radius and zero carry offset.
                    gamepiece.pick(new Pose3d(wx, wy, wz, new Rotation3d()), 0.25, new Vec3(0.0, 0.0, 0.0));
                } else if (mode == GamepieceZone.Mode.OUTTAKE || mode == GamepieceZone.Mode.SHOOT) {
                    // Compute simple world-space velocity vector from robot heading and pitch.
                    final double cosPitch = Math.cos(pitch);
                    double vx = exitSpeed * Math.cos(heading) * cosPitch;
                    double vy = exitSpeed * Math.sin(heading) * cosPitch;
                    double vz = exitSpeed * Math.sin(pitch);
                    physicsWorld.outtakeGamepiece(gamepiece.gamepieceIndex(), wx, wy, wz, vx, vy, vz);
                }
            }
        }
    }

    /**
     * Advances the tracked physics world and refreshes gamepiece zones.
     */
    public synchronized void stepPhysics() {
        final double dtSeconds = physicsWorld != null
                ? physicsWorld.getFixedDtSeconds()
                : DEFAULT_STEP_SECONDS;

        for (SimRobot robot : robots.values()) {
            if (robot != null) {
                robot.update(dtSeconds);
                PhysicsBody robotBody = robotBodies.get(robot.getRobotID());
                if (robotBody != null) {
                    // Use the internal odometry pose for physics sync, not getPose() which may return physics body pose
                    Pose2d odometryPose = robot.getOdometryPose();
                    robotBody.setPosition(new Pose3d(odometryPose.getX(), odometryPose.getY(), 0.175,
                            new Rotation3d(0.0, 0.0, odometryPose.getRotation().getRadians())));
                    robotBody.setOrientation(new Rotation3d(0.0, 0.0, odometryPose.getRotation().getRadians()));
                    ChassisSpeeds speeds = robot.getVelocity();
                    robotBody.setLinearVelocity(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            0.0);
                }
            }
        }

        if (physicsWorld != null) {
            physicsWorld.step();
        }

        updateGamepieceZones();
    }
}
