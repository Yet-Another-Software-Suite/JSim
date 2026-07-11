# API Usage

## Core objects

| Class | Role |
|---|---|
| [`SimWorld`](api/sim_world.md) | Top-level entry point. Owns all bodies and advances the simulation. |
| [`SimBodyBuilder`](api/sim_body_builder.md) | Fluent builder for configuring a body before adding it to the world. |
| [`SimBody`](api/sim_body.md) | Handle to a registered body. Read/write pose and velocity; access the actuator. |
| [`BodyTracker`](api/body_tracker.md) | Optional utility for recording pose/velocity history and distance traveled. |

## Maple-style simulation layers

| Class | Role |
|---|---|
| `SimulatedArena` | Arena lifecycle wrapper that updates drivetrain hooks, field simulation, projectiles, and battery publishing each tick. |
| `SeasonRegistry` + `SeasonConfig` | Versioned season-pack lookup with alias-based migration compatibility. |
| `IntakeSimulation` | Intake helper for nearest-piece acquisition and ejection built on `GamePieceGripper`. |
| `GamePieceProjectile` | Projectile lifecycle/despawn wrapper for spawned game pieces. |
| `DriveTrainSimulation` / `OpponentSimulation` | WPILib supplier-based hooks for robot and opponent chassis motion. |
| `MotorSimulation` + `SimulatedBattery` | Deterministic motor-current and battery-voltage simulation primitives. |

---

## Lifecycle pattern

```java
// ── Construction (robotInit / subsystem constructor) ──────────────────────
SimWorld simWorld = new SimWorld();          // 50 Hz, 10 solver iterations

simWorld.addBody(new SimBodyBuilder("Floor")
    .planeCollider(0, 0, 1, 0)              // +Z normal, at Z=0
    .isStatic()
    .material(Material.CARPET));

SimBody robot = simWorld.addBody(new SimBodyBuilder("Robot")
    .robotId(RobotId.BLUE_1)
    .position(0, 0, 0.1)
    .mass(54)
    .boxCollider(0.3, 0.3, 0.15)
    .noGravity()
    .fixedRotation()
    .material(Material.CARPET));

// ── simulationPeriodic ────────────────────────────────────────────────────
// Push motor outputs into the actuator:
robot.getActuator().setForce(driveForceX, driveForceY, 0);

// Advance physics:
simWorld.step();

// Read back the simulated pose:
Pose3d p = robot.getPose();
```

---

## Collider shapes

| Method | Shape |
|---|---|
| `.sphereCollider(radius)` | Sphere; estimates inertia automatically |
| `.boxCollider(halfX, halfY, halfZ)` | Axis-aligned box in body space; estimates inertia |
| `.planeCollider(nx, ny, nz, offset)` | Infinite half-space; automatically marks body static |

---

## Materials

```java
Material.DEFAULT   // friction 0.5, restitution 0.3
Material.CARPET    // friction 0.8, restitution 0.1  (FRC field)
Material.RUBBER    // friction 0.9, restitution 0.7
Material.STEEL     // friction 0.3, restitution 0.2
Material.ICE       // friction 0.03, restitution 0.1
Material.WOOD      // friction 0.5, restitution 0.4
Material.WALL      // friction 0.2, restitution 0.05
new Material(friction, restitution)  // custom
```

---

## Flags

| Builder method | Effect |
|---|---|
| `.isStatic()` | Infinite mass; never moves |
| `.noGravity()` | Gravity skipped for this body |
| `.noCollision()` | Skipped in collision detection (ghost body) |
| `.fixedRotation()` | Orientation never changes; infinite rotational inertia |

---

## Force generators

JSim provides built-in force generators that you attach to bodies via `SimWorld`:

```java
// Velocity-proportional drag (linear and rotational):
DragForce drag = simWorld.addDragForce(gamePiece, 0.1, 0.05);

// Magnus lift for spinning projectiles:
MagnusForce magnus = simWorld.addMagnusForce(gamePiece, 0.3);

// Direct actuator — set each control loop iteration:
robot.getActuator().setForce(fx, fy, fz);
robot.getActuator().setTorque(tx, ty, tz);
robot.getActuator().zero();     // clear on disable
```

---

## Finding bodies

```java
SimBody floor  = simWorld.findBody("Floor");
SimBody red1   = simWorld.findRobot(RobotId.RED_1);
```

---

## Removing bodies

```java
simWorld.removeBody(gamePiece);
```

This removes the body and its actuator from the simulation. Call this when a game piece exits the field.
