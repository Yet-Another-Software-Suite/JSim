// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.wpilib;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import jsim.api.RobotId;
import jsim.api.SimBody;
import jsim.api.SimBodyBuilder;
import jsim.field.FieldLoader;
import jsim.field.FieldSimulator;
import jsim.field.GamePiece;
import jsim.field.GamePieceGripper;
import jsim.material.Material;

/**
 * A WPILib subsystem that runs a JSim {@link FieldSimulator} automatically.
 *
 * <p>Construct one instance in {@code RobotContainer}, pass suppliers for
 * the robot's chassis speeds and pose, and the subsystem handles everything
 * else: physics stepping, gripper updates, and NT4 game-piece publishing for
 * AdvantageScope.
 *
 * <p>Minimal usage:
 * <pre>{@code
 * private final SimulatedField fieldSim = new SimulatedField(
 *     "2025-reefscape",
 *     drive::getRobotRelativeSpeeds,
 *     drive::getPose);
 * }</pre>
 *
 * <p>For non-default robot dimensions or intake geometry use the builder:
 * <pre>{@code
 * private final SimulatedField fieldSim = SimulatedField.of("2025-reefscape")
 *     .withRobotSize(0.38, 0.38, 0.1)
 *     .withRobotMass(60)
 *     .withIntake(0.4, 0.05, 0.5)
 *     .build(drive::getRobotRelativeSpeeds, drive::getPose);
 * }</pre>
 */
public final class SimulatedField extends SubsystemBase {

    // Defaults sized for a typical 24"×24" competition swerve drive.
    private static final double DEFAULT_ROBOT_HALF_M  = Inches.of(24).in(Meters) / 2;
    private static final double DEFAULT_ROBOT_HALF_Z  = 0.1;
    private static final double DEFAULT_ROBOT_MASS_KG = 54.0;
    private static final double DEFAULT_INTAKE_FWD    = 0.35;
    private static final double DEFAULT_INTAKE_H      = 0.05;
    private static final double DEFAULT_INTAKE_RADIUS = 0.55;
    private static final double DEFAULT_EJECT_XY      = 3.5;
    private static final double DEFAULT_EJECT_Z       = 1.5;

    private final FieldSimulator    fieldSim;
    private final SimBody           simBody;
    private final GamePieceGripper  gripper;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final Supplier<Pose2d>        poseSupplier;
    private final double intakeFwd;
    private final double intakeH;
    private final double intakeRadius;
    private final double ejectXY;
    private final double ejectZ;
    private final Map<String, StructArrayPublisher<Pose3d>> publishers = new HashMap<>();

    /**
     * Create a {@code SimulatedField} with default robot body dimensions (24"×24", 54 kg).
     *
     * <p>The robot body is positioned at the pose returned by {@code poseSupplier} at
     * construction time, so it starts at the same location as the swerve odometry.
     *
     * @param fieldResource  bundled field resource name (e.g. {@code "2025-reefscape"})
     * @param speedsSupplier supplier of robot-relative chassis speeds from swerve odometry
     * @param poseSupplier   supplier of the current robot pose from swerve odometry
     */
    public SimulatedField(String fieldResource,
                          Supplier<ChassisSpeeds> speedsSupplier,
                          Supplier<Pose2d> poseSupplier) {
        this(FieldLoader.fromResource(fieldResource), speedsSupplier, poseSupplier,
             DEFAULT_ROBOT_HALF_M, DEFAULT_ROBOT_HALF_M, DEFAULT_ROBOT_HALF_Z,
             DEFAULT_ROBOT_MASS_KG,
             DEFAULT_INTAKE_FWD, DEFAULT_INTAKE_H, DEFAULT_INTAKE_RADIUS,
             DEFAULT_EJECT_XY, DEFAULT_EJECT_Z);
    }

    private SimulatedField(FieldSimulator fieldSim,
                           Supplier<ChassisSpeeds> speedsSupplier,
                           Supplier<Pose2d> poseSupplier,
                           double robotHalfX, double robotHalfY, double robotHalfZ,
                           double robotMassKg,
                           double intakeFwd, double intakeH, double intakeRadius,
                           double ejectXY, double ejectZ) {
        this.fieldSim       = fieldSim;
        this.speedsSupplier = speedsSupplier;
        this.poseSupplier   = poseSupplier;
        this.intakeFwd      = intakeFwd;
        this.intakeH        = intakeH;
        this.intakeRadius   = intakeRadius;
        this.ejectXY        = ejectXY;
        this.ejectZ         = ejectZ;

        Pose2d start = poseSupplier.get();
        simBody = fieldSim.addRobot(new SimBodyBuilder("Robot")
                .robotId(RobotId.BLUE_1)
                .position(start.getX(), start.getY(), robotHalfZ)
                .mass(Kilograms.of(robotMassKg))
                .boxCollider(robotHalfX, robotHalfY, robotHalfZ)
                .noGravity()
                .fixedRotation()
                .material(Material.CARPET));

        gripper = fieldSim.createGripper(simBody, new Translation3d(intakeFwd, 0, intakeH));

        var nt = NetworkTableInstance.getDefault();
        for (GamePiece gp : fieldSim.getGamePieces()) {
            publishers.computeIfAbsent(gp.getVariant(), variant ->
                    nt.getStructArrayTopic("FieldSimulation/" + variant, Pose3d.struct).publish());
        }
    }

    /**
     * Grab the closest free game piece within intake range. No-op if the gripper is
     * already holding a piece.
     *
     * @return a one-shot {@link Command} suitable for binding to a trigger
     */
    public Command intakeNearest() {
        return runOnce(() -> {
            if (gripper.isHolding()) return;
            Pose3d robotPose = simBody.getPose();
            Translation3d intakePt = new Translation3d(intakeFwd, 0, intakeH)
                    .rotateBy(robotPose.getRotation())
                    .plus(robotPose.getTranslation());
            GamePiece closest = null;
            double minDist = intakeRadius;
            for (GamePiece gp : fieldSim.getGamePieces()) {
                if (gp.isHeld()) continue;
                double dist = gp.getPose().getTranslation().getDistance(intakePt);
                if (dist < minDist) { minDist = dist; closest = gp; }
            }
            if (closest != null) gripper.grab(closest);
        });
    }

    /**
     * Eject the held game piece forward in the robot's heading direction with an
     * upward loft. No-op if nothing is held.
     *
     * @return a one-shot {@link Command} suitable for binding to a trigger
     */
    public Command ejectPiece() {
        return runOnce(() -> {
            Rotation2d heading = simBody.getPose().getRotation().toRotation2d();
            gripper.eject(heading.getCos() * ejectXY, heading.getSin() * ejectXY, ejectZ);
        });
    }

    /**
     * Returns the underlying {@link FieldSimulator} for advanced use — spawning
     * additional game pieces, custom force generators, sensors, etc.
     *
     * @return the underlying field simulator
     */
    public FieldSimulator getFieldSimulator() { return fieldSim; }

    @Override
    public void simulationPeriodic() {
        ChassisSpeeds speeds  = speedsSupplier.get();
        Rotation2d    heading = poseSupplier.get().getRotation();
        double cosH = heading.getCos();
        double sinH = heading.getSin();
        simBody.setLinearVelocity(
                cosH * speeds.vxMetersPerSecond - sinH * speeds.vyMetersPerSecond,
                sinH * speeds.vxMetersPerSecond + cosH * speeds.vyMetersPerSecond,
                0);
        simBody.setAngularVelocity(0, 0, speeds.omegaRadiansPerSecond);

        fieldSim.step();

        for (var entry : publishers.entrySet()) {
            entry.getValue().set(fieldSim.getGamePiecePoses(entry.getKey()));
        }
    }

    /**
     * Entry point for the builder API when non-default robot or intake geometry is needed.
     *
     * @param fieldResource bundled field resource name (e.g. {@code "2025-reefscape"})
     * @return a new {@link Builder}
     */
    public static Builder of(String fieldResource) {
        return new Builder(fieldResource);
    }

    /**
     * Builder for {@link SimulatedField} that allows customizing robot body dimensions,
     * mass, and intake geometry.  All fields default to the same values used by the
     * simple constructor.
     */
    public static final class Builder {
        private final String fieldResource;
        private double robotHalfX   = DEFAULT_ROBOT_HALF_M;
        private double robotHalfY   = DEFAULT_ROBOT_HALF_M;
        private double robotHalfZ   = DEFAULT_ROBOT_HALF_Z;
        private double robotMassKg  = DEFAULT_ROBOT_MASS_KG;
        private double intakeFwd    = DEFAULT_INTAKE_FWD;
        private double intakeH      = DEFAULT_INTAKE_H;
        private double intakeRadius = DEFAULT_INTAKE_RADIUS;
        private double ejectXY      = DEFAULT_EJECT_XY;
        private double ejectZ       = DEFAULT_EJECT_Z;

        private Builder(String fieldResource) {
            this.fieldResource = fieldResource;
        }

        /**
         * Override the robot chassis half-extents (metres).
         *
         * @param halfXMeters half-length along X (forward/back)
         * @param halfYMeters half-width along Y (left/right)
         * @param halfZMeters half-height along Z
         * @return this builder
         */
        public Builder withRobotSize(double halfXMeters, double halfYMeters, double halfZMeters) {
            robotHalfX = halfXMeters;
            robotHalfY = halfYMeters;
            robotHalfZ = halfZMeters;
            return this;
        }

        /**
         * Override the robot mass.
         *
         * @param massKg total robot mass in kilograms
         * @return this builder
         */
        public Builder withRobotMass(double massKg) {
            robotMassKg = massKg;
            return this;
        }

        /**
         * Override the intake geometry.
         *
         * @param forwardMeters distance from robot centre to intake point along the robot's X axis
         * @param heightMeters  height of the intake point above the carpet
         * @param radiusMeters  grab radius — any free piece within this distance is grabbable
         * @return this builder
         */
        public Builder withIntake(double forwardMeters, double heightMeters, double radiusMeters) {
            intakeFwd    = forwardMeters;
            intakeH      = heightMeters;
            intakeRadius = radiusMeters;
            return this;
        }

        /**
         * Override the eject velocity.
         *
         * @param xyMetersPerSec forward speed in the robot's heading direction
         * @param zMetersPerSec  upward loft speed
         * @return this builder
         */
        public Builder withEjectSpeed(double xyMetersPerSec, double zMetersPerSec) {
            ejectXY = xyMetersPerSec;
            ejectZ  = zMetersPerSec;
            return this;
        }

        /**
         * Build the {@link SimulatedField} subsystem.
         *
         * @param speedsSupplier supplier of robot-relative chassis speeds
         * @param poseSupplier   supplier of the current robot pose
         * @return a new {@link SimulatedField} registered with the CommandScheduler
         */
        public SimulatedField build(Supplier<ChassisSpeeds> speedsSupplier,
                                    Supplier<Pose2d> poseSupplier) {
            return new SimulatedField(FieldLoader.fromResource(fieldResource),
                    speedsSupplier, poseSupplier,
                    robotHalfX, robotHalfY, robotHalfZ, robotMassKg,
                    intakeFwd, intakeH, intakeRadius, ejectXY, ejectZ);
        }
    }
}
