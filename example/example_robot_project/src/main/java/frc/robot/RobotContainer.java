// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Built from PathPlanner's AutoBuilder once the swerve subsystem has configured it, so autos
  // discovered in deploy/pathplanner/autos (e.g. "New Auto") show up automatically.
  private final SendableChooser<Command> m_autoChooser;

  // Toggled by a button press to switch the drive stream between angular velocity (right stick X
  // rotates) and heading (right stick X/Y picks the desired heading angle) control.
  private boolean m_headingControlEnabled = false;

  private final SwerveInputStream m_driveStream = m_swerveSubsystem.getAngularVelocityStream(m_driverController::getLeftY,
                                                                                             m_driverController::getLeftX,
                                                                                ()->m_driverController.getRawAxis(2))
                                                      .withControllerHeadingAxis(m_driverController::getRightX,
                                                                                 m_driverController::getRightY)
                                                      .withHeadingControl(() -> m_headingControlEnabled)
                                                      .withDeadband(0.05)
                                                      .withAllianceRelativeControl();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.drive(m_driveStream));
    m_driverController.x().whileTrue(
        m_swerveSubsystem.driveToPointPathPlanner(new Pose2d(Meters.of(3), Meters.of(3), Rotation2d.fromDegrees(180))));
    m_driverController.y().whileTrue(
        m_swerveSubsystem.driveToPointYAMS(new Pose2d(Meters.of(3), Meters.of(3), Rotation2d.fromDegrees(180))));
    m_driverController.start().and(m_driverController.back()).onTrue(m_swerveSubsystem.zeroGyro());
    m_driverController.a().toggleOnTrue(
        Commands.startEnd(() -> m_headingControlEnabled = true, () -> m_headingControlEnabled = false));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autonomous command
    return m_autoChooser.getSelected();
  }
}
