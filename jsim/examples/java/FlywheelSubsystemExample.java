package examples.java;

import jsim.api.GamepieceZone;
import jsim.api.SimRobot;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * FlywheelSubsystem simulates a basic flywheel shooter with a flywheel powerd by two motors facing each other.
 * This is a common FRC design for launching balls.
 */
public class FlywheelSubsystemExample {
  private static final Translation3d[] FLYWHEEL_ZONE_POINTS = {
    new Translation3d(0.0, 0.0, 0.0),
    new Translation3d(0.25, 0.0, 0.0),
    new Translation3d(0.25, 0.12, 0.0),
    new Translation3d(0.0, 0.12, 0.0)
  };
  private static final Translation3d[] BACKSPIN_ZONE_POINTS = {
    new Translation3d(0.0, 0.0, 0.0),
    new Translation3d(0.2, 0.0, 0.0),
    new Translation3d(0.2, 0.15, 0.0),
    new Translation3d(0.0, 0.15, 0.0)
  };

  private final GamepieceZone flywheelZone;
  private double leftMotorVelocity = 0.0;
  private double rightMotorVelocity = 0.0;
  private Rotation3d exitAngle = Rotation3d.kZero;

  /**
   * @param robot The simulated robot this subsystem is attached to.
   */
  public FlywheelSubsystemExample(SimRobot robot) {
    this.flywheelZone = robot.createGamepieceZone(
        "flywheel",
        GamepieceZone.createZoneDimensions(Rotation3d.kZero, FLYWHEEL_ZONE_POINTS),
        Rotation3d.kZero);
  }

  /**
   * Set the velocities of the left and right flywheel motors (in m/s).
   * @param left Left flywheel velocity
   * @param right Right flywheel velocity
   * @param angle Exit angle for the shot
   */
  public void setFlywheel(double left, double right, Rotation3d angle) {
    this.leftMotorVelocity = left;
    this.rightMotorVelocity = right;
    this.exitAngle = angle;
  }

  /**
   * Fire a game piece using the current flywheel settings.
   * The average of the two flywheel velocities is used for the exit speed.
   */
  public void shoot() {
    double avgVelocity = (leftMotorVelocity + rightMotorVelocity) / 2.0;
    flywheelZone.shoot(avgVelocity, exitAngle);
  }

  /**
   * Stop the flywheel shooter.
   */
  public void stop() {
    flywheelZone.disable();
  }

  public GamepieceZone getGamepieceZone() {
    return flywheelZone;
  }
}

/**
 * FlywheelHoodSubsystem simulates a flywheel shooter with a hood and a compression backspin roller.
 * This is a more advanced shooter that can impart backspin for higher arc shots.
 */
class FlywheelHoodSubsystem {
  private final FlywheelSubsystemExample flywheel;
  private final GamepieceZone backspinRollerZone;
  private double backspinVelocity = 0.0;
  private Rotation3d exitAngle = Rotation3d.kZero;

  /**
   * @param robot The simulated robot this subsystem is attached to.
   */
  public FlywheelHoodSubsystem(SimRobot robot) {
    this.flywheel = new FlywheelSubsystemExample(robot);
    this.backspinRollerZone = robot.createGamepieceZone(
        "backspinRoller",
        GamepieceZone.createZoneDimensions(Rotation3d.kZero, BACKSPIN_ZONE_POINTS),
        Rotation3d.kZero);
  }

  /**
   * Set the flywheel and backspin roller velocities and the exit angle.
   * @param left Left flywheel velocity
   * @param right Right flywheel velocity
   * @param backspin Backspin roller velocity
   * @param angle Exit angle for the shot
   */
  public void setShot(double left, double right, double backspin, Rotation3d angle) {
    flywheel.setFlywheel(left, right, angle);
    this.backspinVelocity = backspin;
    this.exitAngle = angle;
  }

  /**
   * Fire a game piece using the flywheel and backspin roller.
   * The flywheel provides the main exit speed, the roller imparts backspin.
   */
  public void shoot() {
    flywheel.shoot();
    backspinRollerZone.shoot(backspinVelocity, exitAngle);
  }

  /**
   * Stop the shooter and roller.
   */
  public void stop() {
    flywheel.stop();
    backspinRollerZone.disable();
  }

  public FlywheelSubsystemExample getFlywheel() {
    return flywheel;
  }

  public GamepieceZone getBackspinRollerZone() {
    return backspinRollerZone;
  }
}
