// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /**
   * Physical, electrical, and tuning constants for {@link frc.robot.subsystems.SwerveSubsystem}.
   * Module-indexed arrays ({@link #kModuleNames}, {@link #kDriveMotorIds}, etc.) all share the same
   * front-left/front-right/back-left/back-right ordering.
   */
  public static class SwerveConstants {
    public static final String[] kModuleNames = {"frontleft", "frontright", "backleft", "backright"};

    /** Drive motor CAN IDs, indexed to match {@link #kModuleNames}. */
    public static final int[] kDriveMotorIds = {1, 4, 7, 10};
    /** Azimuth (steering) motor CAN IDs, indexed to match {@link #kModuleNames}. */
    public static final int[] kAzimuthMotorIds = {2, 5, 8, 11};
    /** Absolute encoder CAN IDs, indexed to match {@link #kModuleNames}. */
    public static final int[] kEncoderIds = {3, 6, 9, 12};
    public static final int kGyroId = 14;

    /** Module locations relative to robot center, indexed to match {@link #kModuleNames}. */
    public static final Translation2d[] kModuleLocations = {
        new Translation2d(Inches.of(24), Inches.of(24)),
        new Translation2d(Inches.of(24), Inches.of(-24)),
        new Translation2d(Inches.of(-24), Inches.of(24)),
        new Translation2d(Inches.of(-24), Inches.of(-24))
    };

    // 360 deg/s gives comfortable spin speed without overshooting in teleop.
    public static final AngularVelocity kMaxAngularVelocity = DegreesPerSecond.of(360);
    // 1 m/s is a conservative starting cap; raise it once PID and feedforward are tuned.
    public static final LinearVelocity kMaxLinearVelocity = MetersPerSecond.of(4);

    private static final Distance kWheelDiameter = Inches.of(4);

    /**
     * Shared drive motor config template. Every module clones this and attaches itself via
     * {@code .clone().withSubsystem(swerveSubsystem)} in {@code SwerveSubsystem#createModule} --
     * the template itself is never handed to a {@link yams.motorcontrollers.SmartMotorController}
     * directly, since {@code withSubsystem} mutates and must only be called once per config.
     */
    public static final SmartMotorControllerConfig kDriveMotorConfig =
        new SmartMotorControllerConfig()
            .withWheelDiameter(kWheelDiameter)
            .withClosedLoopController(0.3, 0, 0)
            .withGearing(new MechanismGearing(12.75))
            .withFeedforward(new SimpleMotorFeedforward(
                0, 12.0 / (kMaxLinearVelocity.in(MetersPerSecond) / kWheelDiameter.in(Meters)), 0.01))
            .withStatorCurrentLimit(Amps.of(40))
            .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    /** Shared azimuth motor config template; see {@link #kDriveMotorConfig} for the clone-per-module pattern. */
    public static final SmartMotorControllerConfig kAzimuthMotorConfig =
        new SmartMotorControllerConfig()
            .withClosedLoopController(1, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0, 1))
            .withGearing(new MechanismGearing(6.75))
            .withStatorCurrentLimit(Amps.of(20))
            .withTelemetry("angleMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    /** Total robot mass including bumpers and battery, used by the jsim collision physics layer. */
    public static final Mass kRobotMass = Kilograms.of(50.0);

    public static final Pose2d kStartingPose = new Pose2d(3, 3, Rotation2d.fromDegrees(0));

    public static final PIDConstants kPathPlannerTranslationPID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kPathPlannerRotationPID = new PIDConstants(5.0, 0.0, 0.0);

    /** Pathfinding constraints, matching {@link #kMaxLinearVelocity}/{@link #kMaxAngularVelocity}. */
    public static final PathConstraints kPathfindingConstraints = new PathConstraints(
        kMaxLinearVelocity, MetersPerSecondPerSecond.of(2.0),
        kMaxAngularVelocity, DegreesPerSecondPerSecond.of(720));
  }
}
