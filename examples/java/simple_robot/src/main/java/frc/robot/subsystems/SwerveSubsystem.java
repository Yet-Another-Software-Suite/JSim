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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.jsim.api.RobotId;
import frc.jsim.api.SimBody;
import frc.jsim.api.SimBodyBuilder;
import frc.jsim.api.SimWorld;
import frc.jsim.material.Material;
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
    private final Pigeon2     gyro  = new Pigeon2(14);
    private final SwerveDrive drive;
    private final Field2d     field = new Field2d();

    // JSim physics world — owns the floor and the robot body.
    private final SimWorld simWorld;
    // Handle to the robot's physics body, tagged as Blue alliance station 1.
    private final SimBody  simBody;

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
                .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
                .withTranslationController(new PIDController(1, 0, 0))
                .withRotationController(new PIDController(1, 0, 0));
        drive = new SwerveDrive(config);

        // Build the JSim world: carpet floor + the robot body tagged as Blue 1.
        simWorld = new SimWorld();
        simWorld.addBody(new SimBodyBuilder("Floor")
                .planeCollider(0, 0, 1, 0)
                .material(Material.CARPET));
        // 24"×24" chassis (~0.6 m square), centre height 0.1 m, 54 kg typical swerve.
        // noGravity + fixedRotation: we drive kinematically by setting velocity each tick.
        simBody = simWorld.addBody(new SimBodyBuilder("Robot")
                .robotId(RobotId.BLUE_1)
                .position(0, 0, 0.1)
                .mass(Kilograms.of(54))
                .boxCollider(Inches.of(24).in(Meters) / 2,
                             Inches.of(24).in(Meters) / 2,
                             0.1)
                .noGravity()
                .fixedRotation()
                .material(Material.CARPET));

        SmartDashboard.putData("Field", field);
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
        // Rotate to world frame using the current heading reported by the drive.
        ChassisSpeeds speeds = drive.getRobotRelativeSpeed();
        Rotation2d heading = drive.getPose().getRotation();
        double cosH = heading.getCos();
        double sinH = heading.getSin();
        double worldVx = cosH * speeds.vxMetersPerSecond - sinH * speeds.vyMetersPerSecond;
        double worldVy = sinH * speeds.vxMetersPerSecond + cosH * speeds.vyMetersPerSecond;
        simBody.setLinearVelocity(worldVx, worldVy, 0);
        simBody.setAngularVelocity(0, 0, speeds.omegaRadiansPerSecond);

        // Advance the physics simulation one fixed timestep (default 20 ms).
        simWorld.step();

        // Read the physics pose back and feed it to the field widget.
        Pose3d p = simBody.getPose();
        Pose2d jsimPose = new Pose2d(p.getX(), p.getY(), p.getRotation().toRotation2d());
        field.setRobotPose(jsimPose);
    }
}
