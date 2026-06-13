// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.api;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Simulation-side gamepiece interaction zone.
 *
 * <p>Usage example:
 * <pre>{@code
 * // Intake zone — front of robot
 * Transform3d[] intakeDims = GamepieceZone.createZoneDimensions(
 *     Rotation3d.kZero,
 *     new Translation3d(0.35, -0.15, 0.1),
 *     new Translation3d(0.35,  0.15, 0.1),
 *     new Translation3d(0.25,  0.15, 0.1),
 *     new Translation3d(0.25, -0.15, 0.1)
 * );
 * GamepieceZone intake = robot.createGamepieceZone("intake", intakeDims, Rotation3d.kZero);
 * StateManager.getInstance().registerGamepieceZone(intake);
 * intake.intake(MetersPerSecond.of(4.0), Rotation3d.kZero);
 *
 * // Shooter zone — dynamic mode via suppliers
 * GamepieceZone shooter = robot.createGamepieceZone("shooter", shooterDims, Rotation3d.kZero);
 * StateManager.getInstance().registerGamepieceZone(shooter);
 * shooter.configure(
 *     () -> flywheelReady() ? GamepieceZone.Mode.SHOOT : GamepieceZone.Mode.DISABLED,
 *     () -> MetersPerSecond.of(flywheelSurfaceSpeed()),
 *     () -> new Rotation3d(0, Math.toRadians(hoodAngle()), 0)
 * );
 * }</pre>
 */
public class GamepieceZone {
  /**
   * Interaction mode for the zone.
   */
  public enum Mode {
    /** Intake a gamepiece into the robot. */
    INTAKE,
    /** Outtake a gamepiece using a single roller. */
    OUTTAKE,
    /** Outtake a gamepiece using a shooter-style mechanism. */
    SHOOT,
    /** Disable gamepiece interaction. */
    DISABLED
  }

  /**
   * Helper to build an array of transform dimensions from a shared rotation and a list of
   * translations. This is a small convenience for example code and users constructing zones.
   *
   * @param rotation the rotation to apply to every translation
   * @param translations the vertex translations defining the zone polygon
   * @return transforms with the provided rotation applied to each translation
   */
  public static Transform3d[] createZoneDimensions(Rotation3d rotation, Translation3d... translations) {
    Transform3d[] transforms = new Transform3d[translations.length];
    for (int i = 0; i < translations.length; i++) {
      transforms[i] = new Transform3d(translations[i], rotation);
    }
    return transforms;
  }

  private final SimRobot robot;
  private final String name;
  private final Transform3d[] zoneDimensions;
  private final Rotation3d robotRotation;
  private double exitRate = 0.0;
  private LinearVelocity exitVelocity = MetersPerSecond.of(0.0);
  private Rotation3d exitRotation = Rotation3d.kZero;
  private Translation3d exitTranslation = new Translation3d();
  private Mode mode = Mode.DISABLED;
  private Supplier<Mode> modeSupplier;
  private Supplier<LinearVelocity> exitVelocitySupplier;
  private Supplier<Rotation3d> exitRotationSupplier;

  /**
   * Internal rule record used to evaluate boolean enter and exit conditions.
   */
  private static final class Rule {
    final Mode mode;
    final Supplier<Boolean> enter;
    final Supplier<Boolean> exit;
    final Supplier<Double> exitRateMetersPerSecondSupplier;
    final Supplier<Rotation3d> rot;
    final Supplier<Translation3d> trans;

    Rule(
        Mode mode,
        Supplier<Boolean> enter,
        Supplier<Boolean> exit,
        Supplier<Double> exitRateMetersPerSecondSupplier,
        Supplier<Rotation3d> rot,
        Supplier<Translation3d> trans) {
      this.mode = mode;
      this.enter = enter;
      this.exit = exit;
      this.exitRateMetersPerSecondSupplier = exitRateMetersPerSecondSupplier;
      this.rot = rot;
      this.trans = trans;
    }
  }

  private final List<Rule> rules = new ArrayList<>();

  /**
   * Compatibility constructor for example code that passes {@code null}.
   *
   * @param ignored compatibility placeholder
   */
  public GamepieceZone(Object ignored) {
    this(null, null, new Transform3d[0], Rotation3d.kZero, false);
  }

  /**
   * Creates a zone attached to a robot.
   *
   * @param robot the simulated robot that owns this zone
   * @param name the zone name used for retrieval from the robot
   * @param zoneDimensions the zone polygon transforms relative to the robot center
   * @param robotRotation the zone rotation relative to the robot
   */
  GamepieceZone(
      SimRobot robot,
      String name,
      Transform3d[] zoneDimensions,
      Rotation3d robotRotation) {
    this(robot, name, zoneDimensions, robotRotation, true);
  }

  /**
   * Returns the robot that owns this zone.
   *
   * @return the owning robot, or {@code null} for compatibility instances
   */
  public SimRobot getRobot() {
    return robot;
  }

  /**
   * Returns the configured zone name.
   *
   * @return the zone name, or {@code null} if unnamed
   */
  public String getName() {
    return name;
  }

  /**
   * Returns a copy of the zone polygon transforms.
   *
   * @return cloned zone dimension transforms
   */
  public Transform3d[] getZoneDimensions() {
    return zoneDimensions.clone();
  }

  /**
   * Returns the zone rotation relative to the robot frame.
   *
   * @return robot-relative rotation
   */
  public Rotation3d getRobotRotation() {
    return robotRotation;
  }

  /**
   * Sets the zone mode directly.
   *
   * @param mode the new zone mode
   */
  public void setMode(Mode mode) {
    this.mode = mode;
  }

  /**
   * Disables gamepiece interaction for this zone.
   */
  public void disable() {
    setMode(Mode.DISABLED);
  }

  /**
   * Sets exit parameters using a velocity and explicit rotation.
   *
   * @param velocity exit speed
   * @param rotation exit orientation
   */
  public void setExitParameters(LinearVelocity velocity, Rotation3d rotation) {
    this.exitVelocity = velocity;
    this.exitRate = velocity.in(MetersPerSecond);
    this.exitRotation = rotation;
  }

  /**
   * Sets exit parameters using a velocity and zero rotation.
   *
   * @param velocity exit speed
   */
  public void setExitParameters(LinearVelocity velocity) {
    setExitParameters(velocity, Rotation3d.kZero);
  }

  /**
   * Enables intake mode with the provided exit settings.
   *
   * @param velocity exit speed
   * @param rotation exit orientation
   */
  public void intake(LinearVelocity velocity, Rotation3d rotation) {
    setExitParameters(velocity, rotation);
    setMode(Mode.INTAKE);
  }

  /**
   * Enables outtake mode with the provided exit settings.
   *
   * @param velocity exit speed
   * @param rotation exit orientation
   */
  public void outtake(LinearVelocity velocity, Rotation3d rotation) {
    setExitParameters(velocity, rotation);
    setMode(Mode.OUTTAKE);
  }

  /**
   * Enables shoot mode with the provided exit settings.
   *
   * @param velocity exit speed
   * @param rotation exit orientation
   */
  public void shoot(LinearVelocity velocity, Rotation3d rotation) {
    setExitParameters(velocity, rotation);
    setMode(Mode.SHOOT);
  }

  /**
   * Installs suppliers used to refresh the zone state each simulation step.
   *
   * @param modeSupplier supplies the active mode
   * @param exitVelocitySupplier supplies the exit velocity
   * @param exitRotationSupplier supplies the exit rotation
   */
  public void configure(
      Supplier<Mode> modeSupplier,
      Supplier<LinearVelocity> exitVelocitySupplier,
      Supplier<Rotation3d> exitRotationSupplier) {
    this.modeSupplier = modeSupplier;
    this.exitVelocitySupplier = exitVelocitySupplier;
    this.exitRotationSupplier = exitRotationSupplier;
  }

  /**
   * Returns the cached exit rate.
   *
   * @return exit rate as a linear velocity
   */
  public LinearVelocity getExitRate() {
    return getExitVelocity();
  }

  /**
   * Returns the configured or supplier-provided exit velocity.
   *
   * @return exit velocity
   */
  public LinearVelocity getExitVelocity() {
    if (exitVelocitySupplier != null) {
      return exitVelocitySupplier.get();
    }
    return exitVelocity;
  }

  /**
   * Returns the configured or supplier-provided exit rotation.
   *
   * @return exit rotation
   */
  public Rotation3d getExitRotation() {
    if (exitRotationSupplier != null) {
      return exitRotationSupplier.get();
    }
    return exitRotation;
  }

  /**
   * Returns the configured exit translation.
   *
   * @return exit translation
   */
  public Translation3d getExitTranslation() {
    return exitTranslation;
  }

  /**
   * Refreshes the zone mode from the configured supplier, if present.
   */
  public void refresh() {
    if (modeSupplier != null) {
      mode = modeSupplier.get();
    }
  }

  /**
   * Advances the zone state for one simulation tick.
   */
  public void update() {
    refresh();
    evaluate();
  }

  private GamepieceZone(
      SimRobot robot,
      String name,
      Transform3d[] zoneDimensions,
      Rotation3d robotRotation,
      boolean register) {
    this.robot = robot;
    this.name = name;
    this.zoneDimensions = zoneDimensions == null ? new Transform3d[0] : zoneDimensions.clone();
    this.robotRotation = robotRotation;
    if (register) {
      StateManager.getInstance().registerGamepieceZone(this);
    }
  }

  /**
   * Configures a rule-driven zone state.
   *
   * @param mode the mode to activate while the enter condition is true
   * @param enter supplies whether the zone should activate
   * @param exit supplies whether the zone should disable
   * @param exitRateMetersPerSecondSupplier supplies the exit rate in meters per second
   * @param rot supplies the exit rotation
   * @param trans supplies the exit translation
   */
  public void configure(
      Mode mode,
      Supplier<Boolean> enter,
      Supplier<Boolean> exit,
      Supplier<Double> exitRateMetersPerSecondSupplier,
      Supplier<Rotation3d> rot,
      Supplier<Translation3d> trans) {
    rules.add(new Rule(mode, enter, exit, exitRateMetersPerSecondSupplier, rot, trans));
  }

  /**
   * Evaluates rule-driven zone updates.
   *
   * <p>If a rule's enter condition is true, the zone is set to the rule's mode and its exit
   * parameters are refreshed from the configured suppliers. If the exit condition is true, the
   * zone is disabled.
   */
  public void evaluate() {
    for (Rule rule : rules) {
      if (rule.exit != null && rule.exit.get()) {
        disable();
        continue;
      }

      if (rule.enter != null && rule.enter.get()) {
        mode = rule.mode;
        if (rule.exitRateMetersPerSecondSupplier != null) {
          exitRate = rule.exitRateMetersPerSecondSupplier.get();
          exitVelocity = MetersPerSecond.of(exitRate);
        }
        if (rule.rot != null) {
          exitRotation = rule.rot.get();
        }
        if (rule.trans != null) {
          exitTranslation = rule.trans.get();
        }
      }
    }
  }

  /**
   * Returns the currently active zone mode.
   *
   * @return active mode, or the supplier-provided mode when configured
   */
  public Mode getMode() {
    return modeSupplier != null ? modeSupplier.get() : mode;
  }
}
