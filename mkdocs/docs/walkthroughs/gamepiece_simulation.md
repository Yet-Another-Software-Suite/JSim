# Gamepiece Simulation: Intake and Shooting

This walkthrough shows how to simulate gamepiece intake and shooting in JSim using `GamepieceZone`. You will set up a physics world, spawn gamepieces on the field, define intake and shooter zones on your simulated robot, and drive them with supplier-based dynamic control.

---

## Overview

`GamepieceZone` is a physics-driven abstraction that handles gamepiece capture and launching without requiring manual collision detection in your simulation code. You define a polygon zone in the robot-local frame, choose a mode (`INTAKE`, `OUTTAKE`, `SHOOT`, or `DISABLED`), and JSim handles the rest each physics tick.

Key responsibilities of `GamepieceZone`:

- **INTAKE** — scans nearby grounded gamepieces within a capture radius and attaches them to the robot
- **OUTTAKE** — ejects a held gamepiece with a physics velocity
- **SHOOT** — launches a held gamepiece as a full projectile with configurable speed, pitch, and heading
- **DISABLED** — the zone is inert; no interaction occurs

The `StateManager` keeps track of all registered zones and steps them forward each simulation tick when you call `StateManager.getInstance().stepPhysics()`.

---

## Step 1: Create a PhysicsWorld and Spawn Gamepieces

Before any zones can interact with gamepieces, a `PhysicsWorld` must exist and be registered with the `StateManager`.

```java
PhysicsWorld world = JSim.createPhysicsWorld(0.02, true); // 20 ms timestep, gravity enabled
StateManager.getInstance().setPhysicsWorld(world);
```

To populate the field with the default gamepiece layout for a season, use `JSim.initializeField()`:

```java
JSim.initializeField(2026); // spawns the default fuel grid for 2026
```

You can also spawn individual gamepieces manually:

```java
Gamepiece note = world.createGamepiece(GamePieceType.BALL);
note.setPosition(new Translation3d(5.0, 3.0, 0.25));
```

For a custom size and mass:

```java
Gamepiece custom = world.createGamepiece(
    0.12,  // radius in meters
    0.23,  // mass in kg
    0.6    // restitution (bounciness, 0–1)
);
custom.setPosition(new Translation3d(3.0, 1.5, 0.12));
```

To inspect all live gamepieces:

```java
List<Gamepiece> pieces = world.gamepieces();
```

!!! note
    `GamePieceType` values include `BALL`, `FUEL`, `CORAL`, `CUSTOM1`, `CUSTOM2`, `CUSTOM3`, and `CUSTOM4`. Use `CUSTOM*` types when your season's gamepiece does not match a built-in shape.

---

## Step 2: Define an Intake Zone

An intake zone is a polygon in the robot-local frame. Use the static helper `GamepieceZone.createZoneDimensions()` to build the corner array, then attach the zone to your simulated robot.

```java
Transform3d[] intakeDims = GamepieceZone.createZoneDimensions(
    Rotation3d.kZero,                        // rotation offset for the whole zone
    new Translation3d(0.35, -0.15, 0.1),     // front-right corner
    new Translation3d(0.35,  0.15, 0.1),     // front-left corner
    new Translation3d(0.20,  0.15, 0.1),     // rear-left corner
    new Translation3d(0.20, -0.15, 0.1)      // rear-right corner
);

GamepieceZone intake = simRobot.createGamepieceZone("intake", intakeDims, Rotation3d.kZero);
StateManager.getInstance().registerGamepieceZone(intake);
```

Activate the zone when your intake rollers are running, and deactivate it when they stop:

```java
// Activate — captures nearby gamepieces at 3 m/s roller surface speed
intake.intake(MetersPerSecond.of(3.0), Rotation3d.kZero);

// Deactivate
intake.disable();
```

!!! tip
    `INTAKE` mode uses a ~0.5 m capture radius centred on the zone polygon. Tune the polygon corners to match your robot's physical intake geometry so the simulation capture radius lines up with your actual mechanism reach.

---

## Step 3: Define a Shooter Zone

A shooter zone works the same way as an intake zone but is activated with `shoot()`. The exit rotation controls the launch angle: Y rotation is pitch and Z rotation is heading offset from the robot's facing direction.

```java
Transform3d[] shooterDims = GamepieceZone.createZoneDimensions(
    Rotation3d.kZero,
    new Translation3d(0.0, -0.10, 0.55),    // left exit point
    new Translation3d(0.0,  0.10, 0.55)     // right exit point
);

GamepieceZone shooter = simRobot.createGamepieceZone("shooter", shooterDims, Rotation3d.kZero);
StateManager.getInstance().registerGamepieceZone(shooter);
```

Fire the shooter with a 15 m/s exit speed at 45° pitch:

```java
shooter.shoot(
    MetersPerSecond.of(15.0),
    new Rotation3d(0, Math.toRadians(45), 0)  // Y = pitch, Z = heading offset
);

// After the shot is complete, return the zone to idle
shooter.disable();
```

!!! warning
    Call `StateManager.getInstance().registerGamepieceZone(zone)` for every zone you create. Zones that are not registered with the `StateManager` will never have their `update()` called and will not interact with any gamepieces.

---

## Step 4: Dynamic Zone Control with Suppliers

Rather than calling `intake()`, `shoot()`, or `disable()` imperatively, you can wire a zone's mode, speed, and angle to live supplier lambdas. The `StateManager` calls `zone.update()` each tick, which re-evaluates all three suppliers automatically.

```java
shooter.configure(
    () -> flywheelAtSpeed() ? GamepieceZone.Mode.SHOOT : GamepieceZone.Mode.DISABLED,
    () -> MetersPerSecond.of(flywheelSurface.getAsDouble()),
    () -> new Rotation3d(0, Math.toRadians(hoodAngle.getAsDouble()), 0)
);
```

The three supplier arguments are:

| Argument | Type | Purpose |
|----------|------|---------|
| First | `Supplier<GamepieceZone.Mode>` | Current mode (`INTAKE`, `SHOOT`, `OUTTAKE`, or `DISABLED`) |
| Second | `Supplier<LinearVelocity>` | Exit or intake speed |
| Third | `Supplier<Rotation3d>` | Exit angle (pitch + heading) |

Because the suppliers are re-evaluated every tick by `StateManager.stepPhysics()`, the zone automatically responds to changes in flywheel speed, hood angle, or any other subsystem state without additional wiring.

!!! note
    You can mix imperative calls and supplier-based configuration on different zones. Use `configure()` for zones that track live subsystem state (flywheel speed, hood angle), and imperative calls for simpler on/off scenarios.

---

## Step 5: The Simulation Loop Ties It Together

Call `StateManager.getInstance().stepPhysics()` in `simulationPeriodic()` to advance the physics engine and refresh all registered zones each tick.

```java
@Override
public void simulationPeriodic() {
    // Keep the simulated chassis in sync with the drive subsystem
    simRobot.setChassisSpeeds(drive.getRobotRelativeSpeed());

    // Advance physics: steps the world, updates all registered GamepieceZones
    StateManager.getInstance().stepPhysics();
}
```

Each call to `stepPhysics()` does the following in order:

1. Re-evaluates supplier-configured zones via `zone.update()`
2. Runs the physics world timestep (gravity, collisions, projectile arcs)
3. Updates the positions of all live gamepieces

---

## Mode Reference

| Mode | Description |
|------|-------------|
| `INTAKE` | Captures nearby grounded gamepieces within ~0.5 m |
| `OUTTAKE` | Pushes a held gamepiece out with a physics velocity |
| `SHOOT` | Launches with full projectile physics (pitch + heading) |
| `DISABLED` | No interaction |

---

## Tips

- Always call `StateManager.registerGamepieceZone(zone)` after creating a zone, or it will never be updated.
- Zone dimensions are expressed in the robot-local frame. The `Rotation3d` passed to `createZoneDimensions()` or `createGamepieceZone()` rotates the entire zone polygon relative to the robot.
- In the exit `Rotation3d`, **Y is pitch** (angle above horizontal) and **Z is heading offset** from the robot's facing direction.
- `INTAKE` uses a ~0.5 m capture radius; adjust your zone polygon corners to reflect the true reach of your robot's intake mechanism.
- Use `world.gamepieces()` to iterate over all live gamepieces for logging or custom scoring logic.
- To query the current mode of a zone at runtime: `zone.getMode()`.
