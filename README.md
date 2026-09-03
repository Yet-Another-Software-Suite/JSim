# JSim

[![CI: build and tests](https://github.com/Yet-Another-Software-Suite/JSim/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/Yet-Another-Software-Suite/JSim/actions/workflows/ci.yml)

JSim is a physics simulation library for FIRST Robotics Competition (FRC) robots. It provides a
layered simulation pipeline that can add physical field interactions to a WPILib or YAMS swerve
drive, including 2D drivetrain collisions and 3D game-piece simulation.

## Features

- Swerve-drive ground-truth physics with WPILib pose estimation support.
- Layered physics processing through the `PhysicsLayer` interface.
- 2D robot collision handling powered by [dyn4j](https://dyn4j.org/).
- Field geometry for the 2026 REBUILT game, including walls, HUBs, TOWER uprights, and TRENCH
	gates.
- 3D FUEL simulation with gravity, bouncing, rolling, field structures, HUB scoring, robot
	interaction, and intake callbacks.
- NetworkTables publishers for simulated poses, intake zones, and scores.

## Repository Layout

```text
example/
	example_robot_project/   WPILib Java project demonstrating JSim
```

The implementation currently lives in the example project under
`example/example_robot_project/src/main/java/jsim/physics`. The example robot in
`src/main/java/frc/robot` shows how to connect the simulation to a YAMS swerve subsystem.

## Requirements

- Java 17
- A 2026 WPILib installation and GradleRIO environment
- A desktop-capable FRC development environment for simulation

## Build and Test

Run these commands from `example/example_robot_project`:

```sh
./gradlew build
./gradlew test
```

To start the desktop robot simulation with the example project:

```sh
./gradlew simulateJava
```

On Windows, use `gradlew.bat` instead of `./gradlew`.

## Using JSim

Create a `SwerveDrivePhysics` instance for the drivebase, then add the physics layers needed by the
simulation:

```java
physicsSim = new SwerveDrivePhysics(drive)
		.addLayer(new Dyn4jCollisionLayer(robotMass, field))
		.addLayer(new FuelLayer(field));
```

Call `drive.simIterate()` followed by `physicsSim.update()` from the subsystem's
`simulationPeriodic()` method. See
[`SwerveSubsystem.java`](example/example_robot_project/src/main/java/frc/robot/subsystems/SwerveSubsystem.java)
and [`FuelLayer.java`](example/example_robot_project/src/main/java/jsim/physics/layers/FuelLayer.java)
for the complete integration examples.

JSim is under active development. APIs and simulation behavior may change as additional FRC field
elements and physics features are added.
