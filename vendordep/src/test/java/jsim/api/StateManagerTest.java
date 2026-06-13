// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.api;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import jsim.PhysicsWorld;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class StateManagerTest {
  @BeforeEach
  void resetStateManager() {
    StateManager.getInstance().reset();
  }

  @Test
  void stepPhysicsUpdatesRobotAndZoneState() {
    StateManager stateManager = StateManager.getInstance();
    SimRobot robot = stateManager.initializeRobot(
        RobotID.BLUE_2,
        new Pose2d(),
        new Translation2d[] {
          new Translation2d(0.0, 0.0),
          new Translation2d(0.4, 0.0),
          new Translation2d(0.4, 0.4)
        });
    robot.setChassisSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));

    GamepieceZone zone = robot.createGamepieceZone(
        "state-manager-test-zone",
        new Transform3d[] {
          new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d()),
          new Transform3d(new Translation3d(0.2, 0.0, 0.0), new Rotation3d()),
          new Transform3d(new Translation3d(0.2, 0.2, 0.0), new Rotation3d()),
          new Transform3d(new Translation3d(0.0, 0.2, 0.0), new Rotation3d())
        },
        new Rotation3d());

    AtomicBoolean enter = new AtomicBoolean(true);
    AtomicBoolean exit = new AtomicBoolean(false);
    AtomicReference<Double> rate = new AtomicReference<>(3.0);
    AtomicReference<Rotation3d> rotation = new AtomicReference<>(new Rotation3d());
    AtomicReference<Translation3d> translation = new AtomicReference<>(new Translation3d());
    zone.configure(
      GamepieceZone.Mode.OUTTAKE,
      enter::get,
      exit::get,
      rate::get,
      rotation::get,
      translation::get);

    stateManager.stepPhysics();

    assertEquals(0.02, robot.getPose().getX(), 1e-9);
    assertEquals(GamepieceZone.Mode.OUTTAKE, zone.getMode());
    assertEquals(3.0, zone.getExitVelocity().in(MetersPerSecond), 1e-9);
  }

  @Test
  void initializeRobotPreservesFrameVerticesAndCreatesBodyWithWorld() {
    StateManager stateManager = StateManager.getInstance();
    PhysicsWorld world = new PhysicsWorld(0.02, true);
    stateManager.setPhysicsWorld(world);

    Translation2d[] frameDimensions = new Translation2d[] {
      new Translation2d(-0.3, -0.2),
      new Translation2d(0.3, -0.2),
      new Translation2d(0.3, 0.2),
      new Translation2d(-0.3, 0.2)
    };

    SimRobot robot = stateManager.initializeRobot(
        RobotID.BLUE_3,
        new Pose2d(1.0, 2.0, new edu.wpi.first.math.geometry.Rotation2d(0.4)),
        frameDimensions);

    assertEquals(4, robot.getFrameVertices().length);
    assertNotNull(stateManager.getRobotBody(RobotID.BLUE_3));

    robot.setChassisSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));
    stateManager.stepPhysics();

    assertEquals(1.02, robot.getPose().getX(), 1e-9);
  }
}
