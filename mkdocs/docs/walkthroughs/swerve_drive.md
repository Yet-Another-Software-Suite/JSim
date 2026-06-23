# Swerve Drive Simulation

This walkthrough follows the complete working example in `examples/java/simple_robot/`.

## Overview

The example integrates JSim into a command-based swerve subsystem. The physics world models the robot as a box body on a carpet floor. Each `simulationPeriodic()` call:

1. Pushes the YAMS odometry chassis speeds into the physics body as a velocity override
2. Steps the physics world (collision response, floor contact)
3. Reads the resulting pose back and sends it to the Field2d widget

## Key code in `SwerveSubsystem.java`

### World setup (constructor)

```java
simWorld = new SimWorld();

simWorld.addBody(new SimBodyBuilder("Floor")
        .planeCollider(0, 0, 1, 0)
        .material(Material.CARPET));

simBody = simWorld.addBody(new SimBodyBuilder("Robot")
        .robotId(RobotId.BLUE_1)
        .position(0, 0, 0.1)
        .mass(54)
        .boxCollider(Inches.of(24).in(Meters) / 2,
                     Inches.of(24).in(Meters) / 2,
                     0.1)
        .noGravity()
        .fixedRotation()
        .material(Material.CARPET));

SmartDashboard.putData("Field", field);
```

The robot body uses `noGravity()` and `fixedRotation()` because the drivetrain is modeled kinematically — we set velocity directly from odometry rather than simulating individual wheel forces.

### Simulation loop

```java
@Override
public void simulationPeriodic() {
    drive.simIterate();                              // update YAMS motor sim

    ChassisSpeeds speeds = drive.getRobotRelativeSpeed();
    Rotation2d heading = drive.getPose().getRotation();
    double cosH = heading.getCos(), sinH = heading.getSin();

    // Rotate robot-relative speeds to world frame
    simBody.setLinearVelocity(
        cosH * speeds.vxMetersPerSecond - sinH * speeds.vyMetersPerSecond,
        sinH * speeds.vxMetersPerSecond + cosH * speeds.vyMetersPerSecond,
        0);
    simBody.setAngularVelocity(0, 0, speeds.omegaRadiansPerSecond);

    simWorld.step();

    Pose3d p = simBody.getPose();
    field.setRobotPose(new Pose2d(p.getX(), p.getY(), p.getRotation().toRotation2d()));
}
```

## Running the simulation

```bash
cd examples/java/simple_robot
./gradlew simulatejava
```

The WPILib simulation GUI opens. Use the Driver Station to enable the robot in teleop and drive with a connected gamepad.

## Adding game pieces

To add a ball that interacts with the robot:

```java
SimBody ball = simWorld.addBody(new SimBodyBuilder("Ball")
    .position(3, 0, 0.085)           // resting on the floor
    .mass(0.24)
    .sphereCollider(0.085)
    .material(Material.RUBBER));
```

The ball will roll and bounce when the robot drives into it. Track it with a `BodyTracker`:

```java
BodyTracker ballTracker = new BodyTracker(ball, 100);

// In simulationPeriodic(), after world.step():
ballTracker.update();
Pose3d ballPose = ballTracker.getPose();
```
