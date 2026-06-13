package examples.java;

import edu.wpi.first.math.geometry.Rotation3d;
import jsim.api.SimRobot;

/**
 * Simple example wrapper around a generic FlywheelSubsystem.
 */
public class FlywheelExample {
  private final FlywheelSubsystemExample flywheel;

  public FlywheelExample(SimRobot robot) {
    this.flywheel = new FlywheelSubsystemExample(robot);
  }

  public void set(double left, double right, Rotation3d angle) {
    flywheel.setFlywheel(left, right, angle);
  }

  public void fire() {
    flywheel.shoot();
  }

  public void stop() {
    flywheel.stop();
  }

  public FlywheelSubsystemExample getFlywheel() {
    return flywheel;
  }
}
