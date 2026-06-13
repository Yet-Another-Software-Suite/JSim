// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.api;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;

/**
 * Regression tests for {@link GamepieceZone} helper behavior.
 */
class GamepieceZoneTest {
  /**
   * Verifies the convenience helpers update the zone mode and exit parameters together.
   */
  @Test
  void helperMethodsSetModeAndExitParametersTogether() {
    SimRobot robot = SimRobot.createRobot(
        RobotID.BLUE_1,
        new Translation2d[] {
          new Translation2d(0.0, 0.0),
          new Translation2d(0.5, 0.0),
          new Translation2d(0.5, 0.5)
        });
    Transform3d[] zoneDimensions = new Transform3d[] {
      new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)),
      new Transform3d(new Translation3d(0.2, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)),
      new Transform3d(new Translation3d(0.2, 0.1, 0.0), new Rotation3d(0.0, 0.0, 0.0)),
      new Transform3d(new Translation3d(0.0, 0.1, 0.0), new Rotation3d(0.0, 0.0, 0.0))
    };
    Rotation3d robotRotation = new Rotation3d(0.0, 0.0, 0.0);
    GamepieceZone zone = robot.createGamepieceZone("intake", zoneDimensions, robotRotation);

    zone.intake(MetersPerSecond.of(2.5), new Rotation3d(0.1, 0.2, 0.3));

    assertEquals(GamepieceZone.Mode.INTAKE, zone.getMode());
    assertEquals(2.5, zone.getExitVelocity().in(MetersPerSecond), 1e-9);
    assertEquals(0.1, zone.getExitRotation().getX(), 1e-9);
    assertEquals(0.2, zone.getExitRotation().getY(), 1e-9);
    assertEquals(0.3, zone.getExitRotation().getZ(), 1e-9);
    assertEquals(4, zone.getZoneDimensions().length);
    assertSame(zone, robot.getGamepieceZone("intake"));

    zone.disable();

    assertEquals(GamepieceZone.Mode.DISABLED, zone.getMode());
  }

  /**
   * Verifies supplier-backed configuration is refreshed into the zone on demand.
   */
  @Test
  void refreshAppliesConfiguredSuppliers() {
    SimRobot robot = SimRobot.createRobot(
        RobotID.BLUE_1,
        new Translation2d[] {
          new Translation2d(0.0, 0.0),
          new Translation2d(0.5, 0.0),
          new Translation2d(0.5, 0.5)
        });
    GamepieceZone zone = robot.createGamepieceZone(
        "outtake",
        new Transform3d[] {
          new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)),
          new Transform3d(new Translation3d(0.3, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)),
          new Transform3d(new Translation3d(0.3, 0.2, 0.0), new Rotation3d(0.0, 0.0, 0.0)),
          new Transform3d(new Translation3d(0.0, 0.2, 0.0), new Rotation3d(0.0, 0.0, 0.0))
        },
        new Rotation3d(0.0, 0.0, 0.0));
    AtomicReference<GamepieceZone.Mode> mode = new AtomicReference<>(GamepieceZone.Mode.OUTTAKE);
    AtomicReference<LinearVelocity> velocity = new AtomicReference<>(MetersPerSecond.of(4.25));
    AtomicReference<Rotation3d> rotation = new AtomicReference<>(new Rotation3d(1.0, 2.0, 3.0));

    zone.configure(mode::get, velocity::get, rotation::get);
    zone.refresh();

    assertEquals(GamepieceZone.Mode.OUTTAKE, zone.getMode());
    assertEquals(4.25, zone.getExitVelocity().in(MetersPerSecond), 1e-9);
    assertSame(rotation.get(), zone.getExitRotation());

    mode.set(GamepieceZone.Mode.SHOOT);
    velocity.set(MetersPerSecond.of(6.0));
    Rotation3d updatedRotation = new Rotation3d(0.0, 0.0, 1.0);
    rotation.set(updatedRotation);
    zone.refresh();

    assertEquals(GamepieceZone.Mode.SHOOT, zone.getMode());
    assertEquals(6.0, zone.getExitVelocity().in(MetersPerSecond), 1e-9);
    assertSame(updatedRotation, zone.getExitRotation());
  }

  /**
   * Verifies the compatibility shim constructor and rule evaluation API work together.
   */
  @Test
  void compatibilityConstructorAndRulesWork() {
    GamepieceZone zone = new GamepieceZone(null);
    AtomicBoolean enter = new AtomicBoolean(true);
    AtomicBoolean exit = new AtomicBoolean(false);
    AtomicReference<Double> rate = new AtomicReference<>(3.75);
    AtomicReference<Rotation3d> rotation = new AtomicReference<>(new Rotation3d(0.4, 0.5, 0.6));
    AtomicReference<Translation3d> translation = new AtomicReference<>(new Translation3d(0.7, 0.8, 0.9));

    zone.configure(GamepieceZone.Mode.SHOOT, enter::get, exit::get, rate::get, rotation::get, translation::get);
    zone.evaluate();

    assertEquals(GamepieceZone.Mode.SHOOT, zone.getMode());
    assertEquals(3.75, zone.getExitRate().in(MetersPerSecond), 1e-9);
    assertEquals(3.75, zone.getExitVelocity().in(MetersPerSecond), 1e-9);
    assertSame(rotation.get(), zone.getExitRotation());
    assertSame(translation.get(), zone.getExitTranslation());

    enter.set(false);
    exit.set(true);
    zone.evaluate();

    assertEquals(GamepieceZone.Mode.DISABLED, zone.getMode());
  }
}
