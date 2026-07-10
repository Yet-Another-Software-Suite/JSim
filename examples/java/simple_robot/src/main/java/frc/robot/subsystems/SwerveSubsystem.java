// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import jsim.api.RobotId;
import jsim.api.SimBody;
import jsim.api.SimBodyBuilder;
import jsim.field.FieldLoader;
import jsim.field.FieldSimulator;
import jsim.field.GamePiece;
import jsim.field.GamePieceGripper;
import jsim.material.Material;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class SwerveSubsystem extends SubsystemBase
{
    // Intake geometry — front-center of the robot, just above carpet.
    private static final double INTAKE_FORWARD_M = 0.35;
    private static final double INTAKE_HEIGHT_M  = 0.05;
    // Any free game piece whose centre is within this distance of the intake point is grabbable.
    private static final double INTAKE_RADIUS_M  = 0.55;
    // Eject velocity: forward at 3.5 m/s, lofted at 1.5 m/s upward.
    private static final double EJECT_SPEED_XY   = 3.5;
    private static final double EJECT_SPEED_Z    = 1.5;

    private final Pigeon2     gyro  = new Pigeon2(14);
    private final SwerveDrive drive;
    private final Field2d     field = new Field2d();

    // JSim field simulator — owns the FRC perimeter, robot body, and all game pieces.
    private final FieldSimulator               fieldSim;
    private final SimBody                      simBody;
    private final GamePieceGripper             gripper;
    // NT4 Pose3d[] publishers that AdvantageScope reads for game-piece 3D rendering.
    private final StructArrayPublisher<Pose3d> coralPublisher;
    private final StructArrayPublisher<Pose3d> algaePublisher;

    private AngularVelocity maximumChassisSpeedsAngularVelocity = DegreesPerSecond.of(360);
    private LinearVelocity  maximumChassisSpeedsLinearVelocity  = MetersPerSecond.of(1);

    /**
     * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
     * controls.
     *
     * @param translationXScalar Translation in the X direction from [-1,1]
     * @param translationYScalar Translation in the Y direction from [-1,1]
     * @param rotationScalar     Rotation speed from [-1,1]
     * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
     */
    public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier translationXScalar,
                                                      DoubleSupplier translationYScalar,
                                                      DoubleSupplier rotationScalar)
    {
        return new SwerveInputStream(drive, translationXScalar, translationYScalar, rotationScalar)
                .withMaximumAngularVelocity(maximumChassisSpeedsAngularVelocity)
                .withMaximumLinearVelocity(maximumChassisSpeedsLinearVelocity)
                .withDeadband(0.01)
                .withCubeRotationControllerAxis()
                .withCubeTranslationControllerAxis()
                .withAllianceRelativeControl();
    }

    /**
     * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
     * controls.
     *
     * @param translationXScalar Translation in the X direction from [-1,1]
     * @param translationYScalar Translation in the Y direction from [-1,1]
     * @param rotationScalar     Rotation speed from [-1,1]
     * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
     */
    public Supplier<ChassisSpeeds> getSimpleChassisSpeeds(DoubleSupplier translationXScalar,
                                                          DoubleSupplier translationYScalar,
                                                          DoubleSupplier rotationScalar)
    {
        return () -> new ChassisSpeeds(maximumChassisSpeedsLinearVelocity.times(translationXScalar.getAsDouble())
                .in(MetersPerSecond),
                maximumChassisSpeedsLinearVelocity.times(translationYScalar.getAsDouble())
                        .in(MetersPerSecond),
                maximumChassisSpeedsAngularVelocity.times(rotationScalar.getAsDouble())
                        .in(RadiansPerSecond));
    }

    public SwerveModule createModule(SparkMax drive, SparkMax azimuth, CANcoder absoluteEncoder, String moduleName,
                                     Translation2d location)
    {
        MechanismGearing driveGearing   = new MechanismGearing(12.75);
        MechanismGearing azimuthGearing = new MechanismGearing(6.75);
        Distance wheelDiameter = Inches.of(4);
        SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
                .withWheelDiameter(wheelDiameter)
                .withClosedLoopController(0.3, 0, 0)
                .withGearing(driveGearing)
                .withFeedforward(new SimpleMotorFeedforward(0,
                        12.0 / (maximumChassisSpeedsLinearVelocity.in(MetersPerSecond) /
                                wheelDiameter.in(Meters)),
                        0.01))
                .withStatorCurrentLimit(Amps.of(40))
                .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
        SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
                .withClosedLoopController(1, 0, 0)
                .withFeedforward(new SimpleMotorFeedforward(0, 1))
                .withGearing(azimuthGearing)
                .withStatorCurrentLimit(Amps.of(20))
                .withTelemetry("angleMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
        SmartMotorController driveSMC   = new SparkWrapper(drive, DCMotor.getNEO(1), driveCfg);
        SmartMotorController azimuthSMC = new SparkWrapper(azimuth, DCMotor.getNEO(1), azimuthCfg);
        SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
                .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
                .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
                .withLocation(location)
                .withOptimization(true);
        return new SwerveModule(moduleConfig);
    }

    public SwerveSubsystem()
    {
        var fl = createModule(new SparkMax(1, MotorType.kBrushless),
                new SparkMax(2, MotorType.kBrushless),
                new CANcoder(3),
                "frontleft",
                new Translation2d(Inches.of(24).in(Meters), Inches.of(24).in(Meters)));
        var fr = createModule(new SparkMax(4, MotorType.kBrushless),
                new SparkMax(5, MotorType.kBrushless),
                new CANcoder(6),
                "frontright",
                new Translation2d(Inches.of(24).in(Meters), Inches.of(-24).in(Meters)));
        var bl = createModule(new SparkMax(7, MotorType.kBrushless),
                new SparkMax(8, MotorType.kBrushless),
                new CANcoder(9),
                "backleft",
                new Translation2d(Inches.of(-24).in(Meters), Inches.of(24).in(Meters)));
        var br = createModule(new SparkMax(10, MotorType.kBrushless),
                new SparkMax(11, MotorType.kBrushless),
                new CANcoder(12),
                "backright",
                new Translation2d(Inches.of(-24).in(Meters), Inches.of(-24).in(Meters)));
        SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
                .withGyro(gyro.getYaw().asSupplier())
                .withStartingPose(new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(0)))
                .withTranslationController(new PIDController(1, 0, 0))
                .withRotationController(new PIDController(1, 0, 0));
        drive = new SwerveDrive(config);

        // Load the 2025 Reefscape field: perimeter walls, reef hitboxes, and pre-staged
        // game pieces (Coral + Algae) from the bundled JSON resource.
        fieldSim = FieldLoader.fromResource("2025-reefscape");

        // 24"×24" chassis (~0.6 m square), centre height 0.1 m, 54 kg typical swerve.
        // noGravity + fixedRotation: we drive kinematically by setting velocity each tick.
        simBody = fieldSim.addRobot(new SimBodyBuilder("Robot")
                .robotId(RobotId.BLUE_1)
                .position(2.0, 2.0, 0.1)
                .mass(Kilograms.of(54))
                .boxCollider(Inches.of(24).in(Meters) / 2,
                             Inches.of(24).in(Meters) / 2,
                             0.1)
                .noGravity()
                .fixedRotation()
                .material(Material.CARPET));

        // Gripper mounted on the robot's front face, just off the carpet.
        gripper = fieldSim.createGripper(simBody,
                new Translation3d(INTAKE_FORWARD_M, 0, INTAKE_HEIGHT_M));

        // NT4 struct-array publishers — AdvantageScope reads these as Pose3d[] for
        // game-piece rendering. In the 3D view, drag the key onto the field and choose
        // the appropriate game-piece model (Coral / Algae from the 2025 asset pack).
        var nt = NetworkTableInstance.getDefault();
        coralPublisher = nt.getStructArrayTopic("FieldSimulation/Coral", Pose3d.struct).publish();
        algaePublisher = nt.getStructArrayTopic("FieldSimulation/Algae", Pose3d.struct).publish();

        SmartDashboard.putData("Field", field);
    }

    /**
     * Find the closest free game piece within {@link #INTAKE_RADIUS_M} of the robot's intake
     * point and grab it. No-op if the gripper is already holding a piece.
     *
     * @return a one-shot {@link Command} suitable for binding to a button
     */
    public Command intakeNearest()
    {
        return runOnce(() -> {
            if (gripper.isHolding()) return;
            Pose3d robotPose = simBody.getPose();
            Translation3d intakePt = new Translation3d(INTAKE_FORWARD_M, 0, INTAKE_HEIGHT_M)
                    .rotateBy(robotPose.getRotation())
                    .plus(robotPose.getTranslation());
            GamePiece closest = null;
            double minDist = INTAKE_RADIUS_M;
            for (GamePiece gp : fieldSim.getGamePieces()) {
                if (gp.isHeld()) continue;
                double dist = gp.getPose().getTranslation().getDistance(intakePt);
                if (dist < minDist) { minDist = dist; closest = gp; }
            }
            if (closest != null) gripper.grab(closest);
        });
    }

    /**
     * Eject the currently held game piece forward in the robot's heading direction with a
     * slight upward loft. No-op if nothing is held.
     *
     * @return a one-shot {@link Command} suitable for binding to a button
     */
    public Command ejectPiece()
    {
        return runOnce(() -> {
            Rotation2d heading = simBody.getPose().getRotation().toRotation2d();
            gripper.eject(
                    heading.getCos() * EJECT_SPEED_XY,
                    heading.getSin() * EJECT_SPEED_XY,
                    EJECT_SPEED_Z);
        });
    }

    /**
     * Drive the {@link SwerveDrive} object with field relative chassis speeds.
     *
     * @param speedsSupplier Field relative {@link ChassisSpeeds}.
     * @return {@link Command} to run the drive.
     */
    public Command drive(Supplier<ChassisSpeeds> speedsSupplier)
    {
        return run(() -> drive.setFieldRelativeChassisSpeeds(speedsSupplier.get())).withName("Field Oriented Drive");
    }

    public Command setRobotRelativeChassisSpeeds(ChassisSpeeds speeds)
    {
        return run(() -> drive.setRobotRelativeChassisSpeeds(speeds));
    }

    public Command driveToPose(Pose2d pose)
    {
        return drive.driveToPose(pose);
    }

    public Command driveRobotRelative(Supplier<ChassisSpeeds> speedsSupplier)
    {
        return drive.drive(speedsSupplier);
    }

    public Command lock()
    {
        return run(drive::lockPose);
    }

    @Override
    public void periodic()
    {
        drive.updateTelemetry();
        field.setRobotPose(drive.getPose());
    }

    @Override
    public void simulationPeriodic()
    {
        drive.simIterate();

        // Push the odometry chassis speeds (robot-relative) into the physics body.
        ChassisSpeeds speeds = drive.getRobotRelativeSpeed();
        Rotation2d heading = drive.getPose().getRotation();
        double cosH = heading.getCos();
        double sinH = heading.getSin();
        double worldVx = cosH * speeds.vxMetersPerSecond - sinH * speeds.vyMetersPerSecond;
        double worldVy = sinH * speeds.vxMetersPerSecond + cosH * speeds.vyMetersPerSecond;
        simBody.setLinearVelocity(worldVx, worldVy, 0);
        simBody.setAngularVelocity(0, 0, speeds.omegaRadiansPerSecond);

        // Advance physics one fixed timestep; gripper update is included inside step().
        fieldSim.step();

        // Mirror physics robot pose back to the 2D field widget.
        Pose3d p = simBody.getPose();
        Pose2d jsimPose = new Pose2d(p.getX(), p.getY(), p.getRotation().toRotation2d());
        field.getObject("JSimPose").setPose(jsimPose);

        // Publish game-piece pose arrays for AdvantageScope 3D rendering.
        coralPublisher.set(fieldSim.getGamePiecePoses("Coral"));
        algaePublisher.set(fieldSim.getGamePiecePoses("Algae"));
    }
}
