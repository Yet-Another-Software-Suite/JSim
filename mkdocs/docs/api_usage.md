# API Usage

This page is the primary reference for using JSim from Java. It covers the full lifecycle from world creation through robot control, gamepiece handling, and Network Tables publishing.

---

## StateManager — the simulation backbone

`StateManager` is a singleton that owns all robot state, the physics world, and registered gamepiece zones. Every subsystem in JSim ultimately reaches back to this object.

```java
import jsim.api.StateManager;

StateManager sm = StateManager.getInstance();
```

### Registering the physics world

Before any robot bodies are created you must hand the world to `StateManager`. This allows robot bodies to auto-register with the underlying physics engine when they are constructed.

```java
import jsim.JSim;
import jsim.PhysicsWorld;

PhysicsWorld world = JSim.createPhysicsWorld(0.02, false);
StateManager.getInstance().setPhysicsWorld(world);
```

!!! warning "Order matters"
    Call `setPhysicsWorld()` before creating any `SimRobot`. Robots that are created before the world is registered will not have a valid physics body.

### Driving the simulation forward

`StateManager.stepPhysics()` is the single call that advances the entire simulation by one fixed timestep. It internally calls `world.step()`, processes all registered gamepiece zones, and updates every robot's pose from the physics results.

```java
// In your periodic / simulation loop:
StateManager.getInstance().stepPhysics();
```

Call this exactly once per control-loop iteration (typically 20 ms / 50 Hz).

### Resetting state (tests only)

```java
StateManager.getInstance().reset();
```

!!! warning "Test use only"
    `reset()` is intended for unit and integration tests. Do not call it during a match simulation.

---

## SimRobot lifecycle

A `SimRobot` represents one physical robot on the field. It holds a physics body and exposes the interface your subsystem code will call.

### Creating a robot

Use either the `JSim` facade or call `SimRobot` directly — both are equivalent.

```java
import jsim.JSim;
import jsim.api.SimRobot;
import jsim.api.RobotID;
import edu.wpi.first.math.geometry.Translation2d;

Translation2d[] frame = {
    new Translation2d( 0.381,  0.381),
    new Translation2d(-0.381,  0.381),
    new Translation2d(-0.381, -0.381),
    new Translation2d( 0.381, -0.381)
};

// Option A — JSim facade (preferred)
SimRobot robot = JSim.createRobot(RobotID.BLUE_1, frame);

// Option B — direct constructor
SimRobot robot = SimRobot.createRobot(RobotID.BLUE_1, frame);
```

`frameDimensions` is an array of `Translation2d` vertices that define the robot's convex hull in robot-relative coordinates (meters).

### RobotID values

| Alliance | Slots |
|----------|-------|
| Blue | `RobotID.BLUE_1`, `RobotID.BLUE_2`, `RobotID.BLUE_3` |
| Red  | `RobotID.RED_1`,  `RobotID.RED_2`,  `RobotID.RED_3`  |

### Setting chassis speeds

Pass a `ChassisSpeeds` to the robot each loop iteration before calling `stepPhysics()`.

```java
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// Example: drive forward at 1 m/s
robot.setChassisSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));
StateManager.getInstance().stepPhysics();
```

### Reading pose

JSim provides two pose accessors with different semantics.

| Method | Returns | Source |
|--------|---------|--------|
| `robot.getPose()` | `Pose2d` | Physics engine result — accounts for collisions, wheel slip, and field boundaries |
| `robot.getOdometryPose()` | `Pose2d` | Commanded (dead-reckoned) pose — same as WPILib odometry, ignores physics |

```java
Pose2d physicsPose    = robot.getPose();
Pose2d odometryPose   = robot.getOdometryPose();
ChassisSpeeds vel     = robot.getVelocity();
```

!!! note "Which pose to use?"
    Use `getPose()` for visualization, collision detection, and anything that should reflect real-world physics. Use `getOdometryPose()` when you want to test odometry algorithms in isolation without physics disturbances.

### Hard-resetting pose

To teleport the robot to a known location (e.g., at the start of autonomous):

```java
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

robot.resetPose(new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(0)));
```

`resetPose()` moves the physics body immediately; the next `getPose()` will reflect the new location.

### Identifying the robot

```java
RobotID id = robot.getRobotID();
```

---

## PhysicsWorld

`PhysicsWorld` wraps the native physics engine and exposes a Java-friendly API for stepping the simulation and managing gamepieces.

### Creating a world

```java
// fixedDtSeconds — timestep length (0.02 = 50 Hz matches the FRC control loop)
// enableGravity  — false for a top-down field sim; true for 3-D trajectory work
PhysicsWorld world = JSim.createPhysicsWorld(0.02, false);
StateManager.getInstance().setPhysicsWorld(world);
```

### `step()` vs `StateManager.stepPhysics()`

| Call | What it does |
|------|--------------|
| `world.step()` | Advances the physics engine by one fixed timestep only |
| `StateManager.getInstance().stepPhysics()` | Calls `world.step()`, then processes all registered gamepiece zones and propagates results to all robots |

**Always prefer `StateManager.stepPhysics()`** in a complete simulation. Call `world.step()` directly only when you need low-level control and are handling zone updates yourself.

### Step listeners

Attach arbitrary callbacks that fire after each physics step — useful for publishing telemetry.

```java
world.addStepListener(() -> {
    // called once per stepPhysics() invocation, after physics resolves
    worldPublisher.publishFrame();
    robotPublisher.publishFrame();
});
```

### Gamepieces

```java
import jsim.api.GamePieceType;

// Spawn a new gamepiece into the world
world.createGamepiece(GamePieceType.NOTE);

// Iterate all live gamepieces
world.gamepieces().forEach(gp -> System.out.println(gp));
```

### Native handle

The native handle is needed when constructing `WorldPosePublisher`.

```java
long handle = world.getNativeHandle();
```

---

## GamepieceZone quick reference

A `GamepieceZone` defines a volume attached to a robot that can capture or eject gamepieces. `StateManager` refreshes every registered zone automatically on each `stepPhysics()` call.

### Creating a zone

```java
import jsim.api.GamepieceZone;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;

Transform3d[] dims = {
    new Transform3d(0.2, 0.0, 0.0, new Rotation3d())
};
Rotation3d zoneOrientation = new Rotation3d(0, 0, 0);

GamepieceZone zone = robot.createGamepieceZone("intake", dims, zoneOrientation);
```

### Registering with StateManager

Zones must be registered to receive automatic updates each tick.

```java
StateManager.getInstance().registerGamepieceZone(zone);
```

### Static mode commands

Call one of the following to set the zone into a fixed operating mode:

```java
import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.MetersPerSecond;

LinearVelocity speed = MetersPerSecond.of(5.0);
Rotation3d angle     = new Rotation3d(0, Math.toRadians(-30), 0);

zone.intake(speed, angle);           // pull gamepieces in
zone.outtake(speed, angle);          // eject onto the field
zone.shoot(speed, angle);            // launch ballistically
zone.disable();                      // zone is inactive
```

### Mode enum

| Constant | Behavior |
|----------|----------|
| `GamepieceZone.Mode.INTAKE` | Captures gamepieces that enter the zone volume |
| `GamepieceZone.Mode.OUTTAKE` | Ejects a held gamepiece at the configured velocity and angle |
| `GamepieceZone.Mode.SHOOT` | Launches a held gamepiece ballistically using the velocity and angle |
| `GamepieceZone.Mode.DISABLED` | Zone is inactive; no capture or ejection |

```java
GamepieceZone.Mode current = zone.getMode();
```

### Dynamic (supplier-based) configuration

When the zone parameters change every loop iteration — for example, an adjustable shooter hood — use `configure()` with suppliers:

```java
import java.util.function.Supplier;

Supplier<GamepieceZone.Mode> modeSupplier = () -> currentMode;
Supplier<LinearVelocity>     velSupplier  = () -> shooterWheel.getSurfaceSpeed();
Supplier<Rotation3d>         rotSupplier  = () -> hoodAngle.get();

zone.configure(modeSupplier, velSupplier, rotSupplier);
```

`StateManager` will call these suppliers on every `stepPhysics()` tick, so the zone always uses the latest values without any additional wiring.

!!! tip "Intake vs. shoot rotation"
    For `OUTTAKE` and `SHOOT`, the simulation uses the rotation's Y component as pitch and the Z component as a heading offset when computing the launch vector. `INTAKE` uses the zone for capture geometry but does not apply the exit rotation.

---

## Field initialization

JSim includes a catalog of official FRC field definitions. Load the field geometry before starting the simulation loop.

```java
JSim.initializeField(2026);
```

### Available years

```java
import jsim.field.FieldDefinitionCatalog;

int[] years = FieldDefinitionCatalog.availableYears();
```

!!! note "Call order"
    Initialize the field after creating the `PhysicsWorld` but before the first `stepPhysics()` call so that field obstacles are present from the first physics step.

---

## Network Tables publishing

JSim ships two publishers that broadcast simulation state over Network Tables for use with tools such as AdvantageScope.

### WorldPosePublisher

Publishes the pose of every physics body in the world each frame.

```java
import jsim.nt.WorldPosePublisher;

long handle = world.getNativeHandle();
WorldPosePublisher worldPub = new WorldPosePublisher(handle, 64); // 64 = max bodies
```

### RobotPosePublisher

Publishes each registered robot's pose each frame.

```java
import jsim.nt.RobotPosePublisher;

RobotPosePublisher robotPub = new RobotPosePublisher();
```

### Wiring publishers as step listeners

The cleanest pattern is to attach both publishers as step listeners so they fire automatically after every physics step without any extra calls in your loop.

```java
world.addStepListener(() -> {
    worldPub.publishFrame();
    robotPub.publishFrame();
});
```

With this pattern your simulation loop becomes:

```java
robot.setChassisSpeeds(speeds);
StateManager.getInstance().stepPhysics(); // physics + zone updates + NT publish
```

### Cleanup

Close both publishers when the simulation ends to release NT resources.

```java
worldPub.close();
robotPub.close();
```

---

## Complete minimal example

The following snippet wires together all of the above in the order JSim expects.

```java
import jsim.JSim;
import jsim.PhysicsWorld;
import jsim.api.StateManager;
import jsim.api.SimRobot;
import jsim.api.RobotID;
import jsim.api.GamepieceZone;
import jsim.api.GamePieceType;
import jsim.nt.WorldPosePublisher;
import jsim.nt.RobotPosePublisher;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class SimExample {

    public static void main(String[] args) throws Exception {

        // 1. Create the physics world (20 ms timestep, no gravity for top-down)
        PhysicsWorld world = JSim.createPhysicsWorld(0.02, false);
        StateManager.getInstance().setPhysicsWorld(world);

        // 2. Load field geometry
        JSim.initializeField(2026);

        // 3. Suppress joystick warnings during sim
        JSim.silenceJoystickWarnings();

        // 4. Create a robot
        Translation2d[] frame = {
            new Translation2d( 0.38,  0.38),
            new Translation2d(-0.38,  0.38),
            new Translation2d(-0.38, -0.38),
            new Translation2d( 0.38, -0.38)
        };
        SimRobot robot = JSim.createRobot(RobotID.BLUE_1, frame);
        robot.resetPose(new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(0)));

        // 5. Add a gamepiece zone
        Transform3d[] zoneDims = { new Transform3d(0.3, 0, 0, new Rotation3d()) };
        GamepieceZone intake = robot.createGamepieceZone("intake", zoneDims, new Rotation3d());
        intake.intake(MetersPerSecond.of(2.0), new Rotation3d());
        StateManager.getInstance().registerGamepieceZone(intake);

        // 6. Wire NT publishers
        WorldPosePublisher worldPub = new WorldPosePublisher(world.getNativeHandle(), 64);
        RobotPosePublisher robotPub = new RobotPosePublisher();
        world.addStepListener(() -> {
            worldPub.publishFrame();
            robotPub.publishFrame();
        });

        // 7. Simulation loop (50 Hz, run for 5 seconds)
        for (int i = 0; i < 250; i++) {
            robot.setChassisSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));
            StateManager.getInstance().stepPhysics();

            Pose2d pose = robot.getPose();
            System.out.printf("t=%.2f  x=%.3f  y=%.3f%n",
                i * 0.02, pose.getX(), pose.getY());
        }

        // 8. Clean up
        worldPub.close();
        robotPub.close();
    }
}
```

!!! tip "FRC Robot class integration"
    In a real FRC project, place `StateManager.getInstance().stepPhysics()` inside `simulationPeriodic()` of your `Robot` class. WPILib calls `simulationPeriodic()` once every 20 ms automatically.
