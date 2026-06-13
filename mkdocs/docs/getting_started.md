# Getting Started

This page walks you through installing JSim, wiring it into a WPILib robot project, and running your first physics simulation.

## Prerequisites

- **Java 17 or newer** — required by WPILib 2025+
- **WPILib 2025+** — JSim targets the 2025 WPILib API surface
- **Gradle** — the standard WPILib robot project build system (the Gradle wrapper bundled with WPILib projects works fine)

!!! note "WPILib installation"
    If you have not installed WPILib yet, follow the [WPILib installation guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) before continuing.

---

## Installation

JSim is distributed as a WPILib vendordep. There are two ways to add it to your project.

### Option A — vendordep JSON (recommended)

In VS Code with WPILib extensions, open the Command Palette and run **WPILib: Manage Vendor Libraries → Install new library (online)**. When prompted for a URL, enter:

```
https://maven.jsim.dev/releases/jsim/jsim-vendordep/vendordep.json
```

WPILib will update `vendordeps/` and `build.gradle` automatically.

### Option B — manual Gradle wiring

Add the JSim Maven repository and dependency directly to `build.gradle`:

```gradle
// build.gradle
repositories {
    maven { url 'https://maven.jsim.dev/releases' }
}
dependencies {
    implementation 'jsim:jsim-vendordep:latest.release'
}
```

!!! tip "Gradle sync"
    After editing `build.gradle`, run `./gradlew build` once so the dependency is resolved and indexed by your IDE.

---

## Minimum Working Example

The snippet below is a self-contained simulation loop you can paste into a test class or a `simulationInit` / `simulationPeriodic` block.

```java
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import jsim.JSim;
import jsim.RobotID;
import jsim.StateManager;
import jsim.robot.SimRobot;
import jsim.world.PhysicsWorld;

public class GettingStartedExample {

    public static void main(String[] args) {
        // 1. Create a physics world with a 20 ms fixed timestep, no gravity.
        PhysicsWorld world = JSim.createPhysicsWorld(0.02, false);

        // 2. Define the robot frame as a rectangle (corner vertices, in metres).
        Translation2d[] frameVertices = {
            new Translation2d( 0.381,  0.381),
            new Translation2d( 0.381, -0.381),
            new Translation2d(-0.381, -0.381),
            new Translation2d(-0.381,  0.381),
        };

        // 3. Create a robot assigned to BLUE alliance slot 1.
        SimRobot robot = JSim.createRobot(RobotID.BLUE_1, frameVertices);

        // 4. Suppress WPILib joystick-not-connected warnings during simulation.
        JSim.silenceJoystickWarnings();

        // 5. Command the robot to drive forward at 1 m/s for 50 ticks (~1 second).
        ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0.0, 0.0);
        robot.setChassisSpeeds(speeds);

        for (int tick = 0; tick < 50; tick++) {
            StateManager.getInstance().stepPhysics();
        }

        // 6. Read the physics-based pose after the run.
        Pose2d pose = robot.getPose();
        System.out.printf("Final pose: x=%.3f m, y=%.3f m, heading=%.1f deg%n",
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees());
    }
}
```

!!! note "Physics pose vs. odometry pose"
    `robot.getPose()` returns the pose calculated by the physics engine and will drift from the commanded trajectory when collisions or other forces act on the robot. `robot.getOdometryPose()` always reflects the purely commanded pose and is useful for comparing expected vs. simulated position.

!!! warning "stepPhysics() call site"
    `StateManager.getInstance().stepPhysics()` must be called once per simulation tick, after all `setChassisSpeeds` calls for that tick. Calling it multiple times per tick or skipping it will produce incorrect physics results.

### Loading a built-in field

To add arena geometry (walls, game elements) matching a competition year, call `initializeField` before stepping physics:

```java
JSim.initializeField(2026); // loads the 2026 Reefscape field geometry
```

### Resetting state between tests

When writing unit tests, call `reset()` at the start of each test to clear all robots and physics state:

```java
StateManager.getInstance().reset();
```

---

## Where to Go Next

| Topic | Page |
|---|---|
| Full API walkthrough | [API Usage](api_usage.md) |
| Swerve drive simulation | [Swerve Drive walkthrough](walkthroughs/swerve_drive.md) |
| Gamepiece simulation | [Gamepiece Simulation walkthrough](walkthroughs/gamepiece_simulation.md) |
| SimRobot API reference | [SimRobot](api/sim_robot.md) |
| StateManager API reference | [StateManager](api/state_manager.md) |
