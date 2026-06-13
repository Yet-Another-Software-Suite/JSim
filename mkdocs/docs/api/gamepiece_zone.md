# GamepieceZone

API reference for `GamepieceZone` — the polygon-based intake/outtake/shoot zone that auto-interacts with nearby gamepieces each physics tick.

---

## Overview

A `GamepieceZone` is a convex polygon attached to a robot that automatically detects and interacts with nearby gamepieces on every `StateManager.stepPhysics()` call. You define the zone's shape in robot-relative coordinates, choose a mode (intake, outtake, shoot, or disabled), and optionally drive the mode and exit parameters from live suppliers so the zone reacts to your robot's mechanism state in real time.

### Setup

```java
// 1. Create the zone on your SimRobot
GamepieceZone zone = simRobot.createGamepieceZone(
    "intake",              // name — must be unique per robot
    dims,                  // Transform3d[] polygon — see Zone dimension helper below
    Rotation3d.kZero       // overall rotation applied to the polygon
);

// 2. Register it with StateManager so it is ticked automatically
StateManager.getInstance().registerGamepieceZone(zone);
```

!!! warning
    A zone that is not registered with `StateManager` will never execute its intake/outtake/shoot logic, even if its mode is set correctly.

---

## Mode enum

| Mode | Behavior |
|------|----------|
| `INTAKE` | Captures grounded gamepieces within approximately 0.5 m of the zone polygon. The gamepiece is attached to the robot at the configured carry offset. |
| `OUTTAKE` | Releases a held gamepiece and pushes it away from the zone with a low exit velocity. |
| `SHOOT` | Launches a held gamepiece with full projectile physics. Exit speed and direction are determined by the configured exit velocity and rotation. |
| `DISABLED` | The zone performs no gamepiece interaction. |

---

## Zone dimension helper

Use `GamepieceZone.createZoneDimensions` to build the `Transform3d[]` polygon array from a list of robot-frame vertices.

```java
Transform3d[] dims = GamepieceZone.createZoneDimensions(
    Rotation3d.kZero,                       // overall rotation offset for the whole polygon
    new Translation3d(0.3,  0.2, 0.1),      // vertex 1 in robot frame (meters)
    new Translation3d(0.3, -0.2, 0.1),      // vertex 2
    new Translation3d(0.5, -0.2, 0.1),      // vertex 3
    new Translation3d(0.5,  0.2, 0.1)       // vertex 4
);

GamepieceZone zone = simRobot.createGamepieceZone("frontIntake", dims, Rotation3d.kZero);
```

!!! tip
    Vertices are specified in robot-frame coordinates using the WPILib convention (X forward, Y left, Z up). The polygon does not need to be planar, but simple flat polygons at a fixed Z work best for ground-level intake zones.

---

## Static control methods

These methods set the zone's mode and exit parameters in one call. Use them when the mode is determined by a one-time command (e.g. a button press or a command group step) rather than a continuously-evaluated condition.

| Method | Description |
|--------|-------------|
| `zone.intake(LinearVelocity speed, Rotation3d angle)` | Switch to `INTAKE` mode and store `speed`/`angle` as exit parameters |
| `zone.outtake(LinearVelocity speed, Rotation3d angle)` | Switch to `OUTTAKE` mode with the given exit parameters |
| `zone.shoot(LinearVelocity speed, Rotation3d angle)` | Switch to `SHOOT` mode with the given exit parameters |
| `zone.disable()` | Switch to `DISABLED` mode; exit parameters are unchanged |
| `zone.setExitParameters(LinearVelocity velocity, Rotation3d rotation)` | Update exit speed and direction without changing the current mode |

---

## Dynamic control via suppliers

For mechanisms whose behavior changes continuously (e.g. a variable-angle shooter or a speed-controlled flywheel), use `configure()` to bind live suppliers. The suppliers are evaluated on every `StateManager.stepPhysics()` tick.

```java
zone.configure(
    // Mode supplier — evaluated each tick
    () -> shouldShoot() ? GamepieceZone.Mode.SHOOT : GamepieceZone.Mode.DISABLED,

    // Exit speed supplier (e.g. from flywheel encoder or setpoint)
    () -> MetersPerSecond.of(flywheelSpeed()),

    // Exit rotation supplier (pitch = Y axis, heading offset = Z axis)
    () -> new Rotation3d(0, Math.toRadians(hoodAngle()), 0)
);
```

!!! note
    Calling `configure()` replaces any previously set static mode. After `configure()`, do not call `intake()`, `outtake()`, or `shoot()` — the supplier will override them on the next tick.

---

## Getters

| Method | Return type | Description |
|--------|-------------|-------------|
| `getMode()` | `GamepieceZone.Mode` | The zone's current mode (`INTAKE`, `OUTTAKE`, `SHOOT`, or `DISABLED`) |
| `getExitVelocity()` | `LinearVelocity` | The configured exit speed |
| `getExitRotation()` | `Rotation3d` | The configured exit rotation (see Exit vector geometry below) |
| `getExitTranslation()` | `Translation3d` | The exit point offset in robot frame |
| `getRobot()` | `SimRobot` | The `SimRobot` this zone is attached to |
| `getName()` | `String` | The name assigned at creation |
| `getZoneDimensions()` | `Transform3d[]` | The polygon vertex array |
| `getRobotRotation()` | `Rotation3d` | The overall rotation applied to the zone polygon |

---

## Exit vector geometry

When the zone fires in `SHOOT` or `OUTTAKE` mode, the gamepiece's initial velocity vector is computed from three components:

| Component | Source | Meaning |
|-----------|--------|---------|
| Speed | `getExitVelocity()` | Magnitude of the exit velocity in m/s |
| Pitch | `getExitRotation().getY()` | Angle above horizontal, in radians (positive = upward) |
| Heading offset | `getExitRotation().getZ()` | Lateral offset from the robot's facing direction, in radians (positive = left) |

The launch point is the robot's current field pose translated by the zone's rotation and `getExitTranslation()`.

```
exit_point   = robot_pose + zone_rotation + exit_translation
exit_heading = robot_heading + exitRotation.getZ()
exit_vector  = speed * (cos(pitch) * field_forward(exit_heading),
                        cos(pitch) * field_left(exit_heading),
                        sin(pitch))
```

!!! tip
    To shoot straight ahead at 30 degrees above horizontal:
    ```java
    zone.shoot(
        MetersPerSecond.of(12.0),
        new Rotation3d(0, Math.toRadians(30), 0)
    );
    ```
    To add a 10-degree rightward heading correction:
    ```java
    new Rotation3d(0, Math.toRadians(30), Math.toRadians(-10))
    ```

---

## Full example

```java
// --- Setup (robot init) ---
Transform3d[] shooterDims = GamepieceZone.createZoneDimensions(
    Rotation3d.kZero,
    new Translation3d(0.35,  0.1, 0.85),
    new Translation3d(0.35, -0.1, 0.85),
    new Translation3d(0.45, -0.1, 0.85),
    new Translation3d(0.45,  0.1, 0.85)
);

GamepieceZone shooter = simRobot.createGamepieceZone("shooter", shooterDims, Rotation3d.kZero);
StateManager.getInstance().registerGamepieceZone(shooter);

// Bind suppliers so the zone always reflects real mechanism state
shooter.configure(
    () -> flywheelRunning() ? GamepieceZone.Mode.SHOOT : GamepieceZone.Mode.DISABLED,
    () -> MetersPerSecond.of(flywheelRPMToMps(flywheelRPM.get())),
    () -> new Rotation3d(0, Math.toRadians(hoodAngleDeg.get()), 0)
);

// --- Separate intake zone ---
Transform3d[] intakeDims = GamepieceZone.createZoneDimensions(
    Rotation3d.kZero,
    new Translation3d(0.5,  0.15, 0.05),
    new Translation3d(0.5, -0.15, 0.05),
    new Translation3d(0.7, -0.15, 0.05),
    new Translation3d(0.7,  0.15, 0.05)
);

GamepieceZone intake = simRobot.createGamepieceZone("groundIntake", intakeDims, Rotation3d.kZero);
StateManager.getInstance().registerGamepieceZone(intake);

// Static mode — toggled by commands
intake.intake(MetersPerSecond.of(0), Rotation3d.kZero);
```
