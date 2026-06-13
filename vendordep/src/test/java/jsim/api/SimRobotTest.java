// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.api;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import jsim.PhysicsBody;
import jsim.PhysicsWorld;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SimRobotTest {
  @BeforeEach
  void resetStateManager() {
    StateManager.getInstance().reset();
  }

  @Test
  void getVelocityFallsBackToCommandedSpeedsWhenNoBodyExists() {
    SimRobot robot = SimRobot.createRobot(
        RobotID.RED_1,
        new Translation2d[] {
          new Translation2d(0.0, 0.0),
          new Translation2d(0.4, 0.0),
          new Translation2d(0.4, 0.4)
        });

    robot.setChassisSpeeds(new ChassisSpeeds(1.0, -0.5, 0.25));

    assertEquals(1.0, robot.getVelocity().vxMetersPerSecond, 1e-9);
    assertEquals(-0.5, robot.getVelocity().vyMetersPerSecond, 1e-9);
    assertEquals(0.25, robot.getVelocity().omegaRadiansPerSecond, 1e-9);
  }

  @Test
  void getVelocityReadsPhysicsBodyWhenAttached() {
    StateManager stateManager = StateManager.getInstance();
    PhysicsWorld world = new PhysicsWorld(0.02, true);
    stateManager.setPhysicsWorld(world);

    SimRobot robot = stateManager.initializeRobot(
        RobotID.RED_2,
        new Pose2d(),
        new Translation2d[] {
          new Translation2d(0.0, 0.0),
          new Translation2d(0.4, 0.0),
          new Translation2d(0.4, 0.4)
        });
    PhysicsBody body = world.createBody(20.0);
    body.setLinearVelocity(2.5, -1.25, 0.0);
    stateManager.setRobotBody(RobotID.RED_2, body);

    assertEquals(2.5, robot.getVelocity().vxMetersPerSecond, 1e-9);
    assertEquals(-1.25, robot.getVelocity().vyMetersPerSecond, 1e-9);
    assertEquals(0.0, robot.getVelocity().omegaRadiansPerSecond, 1e-9);
  }
}
