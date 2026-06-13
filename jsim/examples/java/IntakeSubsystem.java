package examples.java;

import jsim.api.GamepieceZone;
import edu.wpi.first.math.geometry.Rotation3d;
import jsim.api.SimRobot;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * A subsystem representing a robot's intake mechanism.
 *
 * <p>This class uses a {@link GamepieceZone} to simulate the intake and outtake of game pieces. It
 * can be set to different modes for collecting game pieces from the field or ejecting them.
 */
public class IntakeSubsystem {
  private static final Translation3d[] INTAKE_ZONE_POINTS = {
    new Translation3d(0.0, 0.0, 0.0),
    new Translation3d(0.22, 0.0, 0.0),
    new Translation3d(0.22, 0.16, 0.0),
    new Translation3d(0.0, 0.16, 0.0)
  };

  private final GamepieceZone intakeZone;

  /**
   * Enum representing the different states of the intake.
   */
  public enum IntakeMode {
    /** The intake is off and not interacting with game pieces. */
    DISABLED,
    /** The intake is actively pulling game pieces in. */
    INTAKE,
    /** The intake is actively pushing game pieces out. */
    OUTTAKE
  }

  /**
   * Constructs a new IntakeSubsystem.
   *
   * @param robot The {@link SimRobot} this subsystem is a part of.
   * @param intakeSpeed The speed (in meters per second) at which the intake will pull in or push
   *     out game pieces. A positive value is used for both intake and outtake directions.
   */
  public IntakeSubsystem(SimRobot robot, double intakeSpeed) {
    this.intakeZone = robot.createGamepieceZone(
        "intake",
      GamepieceZone.createZoneDimensions(Rotation3d.kZero, INTAKE_ZONE_POINTS),
        Rotation3d.kZero);
    // Configure the zone for intake/outtake behavior.
    // The rotation is typically zero, as it's a direct pickup/ejection.
    // The velocity is the speed of the rollers.
    this.intakeZone.setExitParameters(intakeSpeed);
  }

  /**
   * Sets the current mode of the intake.
   *
   * @param mode The {@link IntakeMode} to set.
   */
  public void setMode(IntakeMode mode) {
    switch (mode) {
      case INTAKE:
        intakeZone.setMode(GamepieceZone.Mode.INTAKE);
        break;
      case OUTTAKE:
        intakeZone.setMode(GamepieceZone.Mode.OUTTAKE);
        break;
      case DISABLED:
      default:
        intakeZone.disable();
        break;
    }
  }

  /**
   * Gets the underlying GamepieceZone instance.
   *
   * @return The GamepieceZone used by this subsystem.
   */
  public GamepieceZone getGamepieceZone() {
    return intakeZone;
  }
}
