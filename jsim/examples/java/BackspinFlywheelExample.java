package examples.java;

import jsim.api.GamepieceZone;
import edu.wpi.first.math.geometry.Rotation3d;
import jsim.api.SimRobot;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Example that adds a compression/backspin roller to a generic flywheel.
 */
public class BackspinFlywheelExample {
  private static final Translation3d[] BACKSPIN_ZONE_POINTS = {
    new Translation3d(0.0, 0.0, 0.0),
    new Translation3d(0.2, 0.0, 0.0),
    new Translation3d(0.2, 0.15, 0.0),
    new Translation3d(0.0, 0.15, 0.0)
  };

  private final FlywheelSubsystemExample flywheel;
  private final GamepieceZone backspinRollerZone;
  private double backspinVelocity = 0.0;
  private Rotation3d exitAngle = Rotation3d.kZero;

  public BackspinFlywheelExample(SimRobot robot) {
    this.flywheel = new FlywheelSubsystemExample(robot);
    this.backspinRollerZone = robot.createGamepieceZone(
      "backspinRoller",
      GamepieceZone.createZoneDimensions(Rotation3d.kZero, BACKSPIN_ZONE_POINTS),
      Rotation3d.kZero);
  }

  public void setShot(double left, double right, double backspin, Rotation3d angle) {
    flywheel.setFlywheel(left, right, angle);
    this.backspinVelocity = backspin;
    this.exitAngle = angle;
  }

  public void fire() {
    flywheel.shoot();
    backspinRollerZone.setExitParameters(backspinVelocity, exitAngle);
    backspinRollerZone.setMode(GamepieceZone.Mode.SHOOT);
  }

  public void stop() {
    flywheel.stop();
    backspinRollerZone.setMode(GamepieceZone.Mode.DISABLED);
  }

  public FlywheelSubsystemExample getFlywheel() {
    return flywheel;
  }

  public GamepieceZone getBackspinRollerZone() {
    return backspinRollerZone;
  }
}
