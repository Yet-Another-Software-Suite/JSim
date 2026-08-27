// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import jsim.physics.SwerveDrivePhysics;
import jsim.physics.layers.Dyn4jCollisionLayer;
import jsim.physics.layers.fields.Field2026;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Swerve drive subsystem built with YAMS. This example shows how to wire up four NEO-driven modules
 * (each with a drive motor, a steer motor, and a CANcoder absolute encoder) into a field-relative
 * SwerveDrive, along with the teleop control stream and PathPlanner autonomous integration used
 * across YASS example projects.
 *
 * <p>Physical/electrical/tuning constants -- including the shared drive and azimuth motor
 * {@link SmartMotorControllerConfig} templates -- live in {@link SwerveConstants}; this class is
 * just the wiring (constructing hardware objects and attaching them to those configs) plus the
 * control-command surface. {@code SwerveDriveConfig} is assembled here rather than in Constants
 * since it needs the live {@link SwerveModule}s built from real hardware.
 */
public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive drive;
  private final Pigeon2 gyro;
  private final SwerveDrivePhysics physicsSim;

  /**
   * Builds a {@link SwerveInputStream} from joystick axes, pre-capped at {@link
   * SwerveConstants#kMaxLinearVelocity}/{@link SwerveConstants#kMaxAngularVelocity} (the stream's
   * own defaults are a much faster 4 m/s / 1 rotation per second). Callers chain their own
   * deadband, cubing, heading control, or alliance-relative configuration on top (see
   * {@code RobotContainer}).
   */
  public SwerveInputStream getAngularVelocityStream(
      DoubleSupplier translationXScalar,
      DoubleSupplier translationYScalar,
      DoubleSupplier rotationScalar) {
    return new SwerveInputStream(drive, translationXScalar, translationYScalar, rotationScalar)
        .withMaximumLinearVelocity(SwerveConstants.kMaxLinearVelocity)
        .withMaximumAngularVelocity(SwerveConstants.kMaxAngularVelocity);
  }

  public SwerveModule createModule(
      SparkMax drive,
      SparkMax azimuth,
      CANcoder absoluteEncoder,
      String moduleName,
      Translation2d location) {
    SmartMotorControllerConfig driveCfg = SwerveConstants.kDriveMotorConfig.clone().withSubsystem(this).withTelemetry(moduleName + "/drive", TelemetryVerbosity.HIGH);
    SmartMotorControllerConfig azimuthCfg = SwerveConstants.kAzimuthMotorConfig.clone().withSubsystem(this).withTelemetry(moduleName + "/azimuth", TelemetryVerbosity.HIGH);

    SmartMotorController driveSMC = new SparkWrapper(drive, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuth, DCMotor.getNEO(1), azimuthCfg);

    SwerveModuleConfig moduleConfig =
        new SwerveModuleConfig(driveSMC, azimuthSMC)
            .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
            .withTelemetry(moduleName, TelemetryVerbosity.HIGH)
            .withLocation(location)
            .withOptimization(true);

    return new SwerveModule(moduleConfig);
  }

  public SwerveSubsystem() {
    gyro = new Pigeon2(SwerveConstants.kGyroId);

    SwerveModule[] modules = new SwerveModule[SwerveConstants.kModuleNames.length];
    for (int i = 0; i < modules.length; i++) {
      modules[i] = createModule(
          new SparkMax(SwerveConstants.kDriveMotorIds[i], MotorType.kBrushless),
          new SparkMax(SwerveConstants.kAzimuthMotorIds[i], MotorType.kBrushless),
          new CANcoder(SwerveConstants.kEncoderIds[i]),
          SwerveConstants.kModuleNames[i],
          SwerveConstants.kModuleLocations[i]);
    }

    SwerveDriveConfig config =
        new SwerveDriveConfig(this, modules[0], modules[1], modules[2], modules[3])
            .withGyro(gyro.getYaw().asSupplier())
            .withStartingPose(SwerveConstants.kStartingPose)
            .withTranslationController(new PIDController(1,0,0))
            .withRotationController(new PIDController(1,0,0));

    drive = new SwerveDrive(config);
    var field = new Field2026();
    field.field = drive.getField2d();
    physicsSim =
        new SwerveDrivePhysics(drive)
            .addLayer(new Dyn4jCollisionLayer(SwerveConstants.kRobotMass, field));

    configurePathPlanner();
  }

  /**
   * Configures PathPlanner's {@link AutoBuilder} against this drive so autos discovered under
   * {@code deploy/pathplanner/autos} can drive it, and so {@link #driveToPointPathPlanner} can
   * pathfind using {@code deploy/pathplanner/navgrid.json}. Reads robot physical parameters from
   * {@code deploy/pathplanner/settings.json} (edit via the PathPlanner GUI, or by hand to match
   * {@link SwerveConstants}).
   */
  private void configurePathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load PathPlanner GUI settings", e);
    }

    AutoBuilder.configure(
        drive::getPose,
        drive::resetOdometry,
        drive::getRobotRelativeSpeed,
        (speeds, feedforwards) -> drive.setRobotRelativeChassisSpeeds(speeds),
        new PPHolonomicDriveController(
            SwerveConstants.kPathPlannerTranslationPID, SwerveConstants.kPathPlannerRotationPID),
        config,
        () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
        this);
  }

  /**
   * Drive field-relative from a {@link SwerveInputStream} (e.g. joystick axes with deadband,
   * cubing, and/or heading control chained on by the caller).
   *
   * @param stream Field-relative {@link ChassisSpeeds} source.
   * @return {@link Command} that continuously drives from the stream.
   */
  public Command drive(SwerveInputStream stream) {
    return drive.drive(() -> ChassisSpeeds.fromFieldRelativeSpeeds(stream.get(), new Rotation2d(drive.getGyroAngle())));
  }

  public Command setRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
    return run(() -> drive.setRobotRelativeChassisSpeeds(speeds));
  }

  public Command driveRobotRelative(Supplier<ChassisSpeeds> speedsSupplier) {
    return drive.drive(speedsSupplier);
  }

  /**
   * Drive to the given point on the field using YAMS' {@link SwerveDrive#driveToPose(Pose2d)}.
   *
   * @param point Field-relative pose to drive to.
   * @return {@link Command} that drives to the given point.
   */
  public Command driveToPointYAMS(Pose2d point) {
    return drive.driveToPose(point);
  }

  /**
   * Drive to the given point on the field using PathPlanner's on-the-fly pathfinding.
   *
   * @param point Field-relative pose to drive to.
   * @return {@link Command} that pathfinds to the given point.
   */
  public Command driveToPointPathPlanner(Pose2d point) {
    return AutoBuilder.pathfindToPose(point, SwerveConstants.kPathfindingConstraints);
  }

  public Command lock() {
    return run(drive::lockPose);
  }

  /**
   * Zero the gyro, resetting the robot's heading to face away from the driver station.
   *
   * @return {@link Command} that zeroes the gyro.
   */
  public Command zeroGyro() {
    return runOnce(drive::zeroGyro).withName("Zero Gyro");
  }

  /**
   * Gets the measured pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose.
   */
  public Pose2d getPose() {
    return drive.getPose();
  }

  /**
   * Fuse a vision-derived pose measurement into the drive's pose estimator.
   *
   * @param visionPose       Vision-measured {@link Pose2d}, field relative.
   * @param timestampSeconds Timestamp the measurement was taken at, matching
   *                         {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    drive.addVisionMeasurement(visionPose, timestampSeconds);
  }

  @Override
  public void periodic() {
    drive.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    drive.simIterate();
    physicsSim.update();
  }
}