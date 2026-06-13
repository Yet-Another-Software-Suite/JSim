# StateManager

`StateManager` is the **single source of truth** for all simulation state in JSim. It is the only class permitted to mutate `FieldState<T>` objects; every other API surface is a read/write proxy through the methods defined here.

Responsibilities:

- Owns and tracks every [`SimRobot`](sim_robot.md) by `RobotID`
- Holds the active `PhysicsWorld` and maps each robot to its `PhysicsBody`
- Stores field elements loaded from a `FieldConfig`
- Maintains the list of registered `GamepieceZone`s
- Drives the entire simulation forward via `stepPhysics()`

!!! note "Package"
    `jsim.api.StateManager`

---

## Singleton access

`StateManager` is a process-wide singleton. Obtain the instance with:

```java
import jsim.api.StateManager;

StateManager sm = StateManager.getInstance();
```

Do not store the reference in a static field of your own — call `getInstance()` each time, or capture it in a local variable. This keeps the call explicit and makes test isolation (via `reset()`) straightforward.

---

## Robot management

| Method | Returns | Description |
|--------|---------|-------------|
| `initializeRobot(RobotID, Pose2d, Translation2d[])` | `SimRobot` | Creates a new robot, stores its state, and auto-registers a physics body if a world is already attached |
| `getRobot(RobotID)` | `SimRobot` or `null` | Returns the registered robot for the given id, or `null` if none exists |
| `getRobots()` | `Map<RobotID, SimRobot>` (immutable) | Snapshot of all registered robots keyed by `RobotID` |
| `getRobotPoses()` | `Map<RobotID, Pose2d>` (immutable) | Snapshot of all robots' current odometry poses |

```java
// Create robots
SimRobot blue1 = StateManager.getInstance()
    .initializeRobot(RobotID.BLUE_1, new Pose2d(1.5, 5.5, new Rotation2d()), frame);

// Look up later
SimRobot robot = StateManager.getInstance().getRobot(RobotID.BLUE_1);

// Iterate all robots
StateManager.getInstance().getRobots().forEach((id, r) ->
    System.out.println(id + " -> " + r.getPose()));

// Bulk pose read (e.g. for telemetry)
Map<RobotID, Pose2d> poses = StateManager.getInstance().getRobotPoses();
```

!!! tip "Prefer `SimRobot.createRobot()` for typical use"
    `SimRobot.createRobot(RobotID, Translation2d[])` is a thin wrapper around `initializeRobot()` that fills in a zero starting pose. Use `initializeRobot()` directly when you need a non-zero starting pose in a single call.

---

## Physics world

| Method | Returns | Description |
|--------|---------|-------------|
| `setPhysicsWorld(PhysicsWorld)` | `void` | Stores the active physics world; clears existing body mappings and re-registers every already-known robot |
| `getPhysicsWorld()` | `PhysicsWorld` or `null` | Returns the active world, or `null` if none has been set |
| `setRobotBody(RobotID, PhysicsBody)` | `void` | Associates a native `PhysicsBody` with a robot id; pass `null` to clear the mapping |
| `getRobotBody(RobotID)` | `PhysicsBody` or `null` | Returns the physics body for a robot, or `null` if none exists |
| `setPhysicsVelocity(RobotID, ChassisSpeeds)` | `void` | Directly writes a velocity command to the robot's physics body (linear X/Y only) |

```java
import jsim.JSim;
import jsim.PhysicsWorld;

PhysicsWorld world = JSim.createPhysicsWorld(0.02, false);
StateManager.getInstance().setPhysicsWorld(world);

// Read back
PhysicsWorld w = StateManager.getInstance().getPhysicsWorld();
```

!!! warning "Order matters"
    Always call `setPhysicsWorld()` **before** creating any robots. Robots that are initialized before a world is set will have their bodies registered automatically when `setPhysicsWorld()` is eventually called, but physics simulation will not begin for them until that point.

---

## Field initialization

| Method | Returns | Description |
|--------|---------|-------------|
| `initializeField(FieldConfig)` | `void` | Parses a `FieldConfig` and populates the internal list of `FieldElement` objects |
| `getFieldElements()` | `List<FieldElement>` (immutable) | Returns all field elements currently registered in the simulation |

```java
import jsim.JSim;

// Convenience wrapper — loads the 2026 season field
JSim.initializeField(2026);

// Or pass a FieldConfig directly
StateManager.getInstance().initializeField(myConfig);

// Inspect elements
StateManager.getInstance().getFieldElements()
    .forEach(e -> System.out.println(e));
```

!!! note "Call order"
    Initialize the field after creating the `PhysicsWorld` but before the first `stepPhysics()` call so that field obstacles are present from the first physics step.

---

## Gamepiece zones

| Method | Returns | Description |
|--------|---------|-------------|
| `registerGamepieceZone(GamepieceZone)` | `void` | Adds the zone to the list that is refreshed on every `stepPhysics()` call; no-op if the zone is already registered |

```java
GamepieceZone intake = robot.createGamepieceZone("intake", dims, new Rotation3d());
StateManager.getInstance().registerGamepieceZone(intake);
```

Zones are evaluated inside `stepPhysics()` — see the next section for exactly when in the tick they run.

---

## stepPhysics()

`StateManager.getInstance().stepPhysics()` is the **single call** that advances the entire simulation by one fixed timestep. Call it exactly once per control-loop iteration (typically every 20 ms at 50 Hz).

### What happens each tick

1. **Dead-reckoning update** — `robot.update(dt)` is called for every registered robot. This integrates the last `ChassisSpeeds` command into the odometry pose.

2. **Physics body sync** — For each robot that has a `PhysicsBody`, the body's position and linear velocity are overwritten from the freshly updated odometry pose. This drives the body in the physics engine to match the commanded motion.

3. **Physics world step** — `physicsWorld.step()` is called, resolving collisions, applying impulses, and advancing gamepiece trajectories.

4. **Gamepiece zone refresh** — Every registered `GamepieceZone` is updated (suppliers re-evaluated, mode applied). The manager then iterates all live gamepieces and, for each zone:
    - **INTAKE** — attempts to capture any gamepiece within the capture radius or inside the zone polygon.
    - **OUTTAKE / SHOOT** — launches any nearby gamepiece at the configured exit velocity and rotation.
    - **DISABLED** — skipped entirely.

```java
// Minimal loop
robot.setChassisSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));
StateManager.getInstance().stepPhysics();
Pose2d pose = robot.getPose();
```

!!! tip "FRC Robot class integration"
    In a real FRC project, place `StateManager.getInstance().stepPhysics()` inside `simulationPeriodic()` of your `Robot` class. WPILib calls `simulationPeriodic()` once every 20 ms automatically.

---

## reset()

Clears all tracked simulation state: robots, robot bodies, gamepiece zones, field elements, and the physics world reference.

```java
StateManager.getInstance().reset();
```

!!! warning "Test use only"
    `reset()` is intended exclusively for unit and integration tests to ensure deterministic, isolated state between runs. Do **not** call it during a match simulation.

---

## Full simulation loop example

```java
import jsim.JSim;
import jsim.PhysicsWorld;
import jsim.api.StateManager;
import jsim.api.SimRobot;
import jsim.api.RobotID;
import jsim.api.GamepieceZone;
import jsim.nt.WorldPosePublisher;
import jsim.nt.RobotPosePublisher;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class SimLoop {

    public static void main(String[] args) throws Exception {

        // ── 1. Physics world ──────────────────────────────────────────────
        PhysicsWorld world = JSim.createPhysicsWorld(0.02, false);
        StateManager.getInstance().setPhysicsWorld(world);

        // ── 2. Field geometry ─────────────────────────────────────────────
        JSim.initializeField(2026);

        // ── 3. Robot ──────────────────────────────────────────────────────
        Translation2d[] frame = {
            new Translation2d( 0.381,  0.381),
            new Translation2d(-0.381,  0.381),
            new Translation2d(-0.381, -0.381),
            new Translation2d( 0.381, -0.381)
        };
        SimRobot robot = StateManager.getInstance()
            .initializeRobot(RobotID.BLUE_1, new Pose2d(1.5, 5.5, new Rotation2d()), frame);

        // ── 4. Gamepiece zone ─────────────────────────────────────────────
        Transform3d[] zoneDims = { new Transform3d(0.3, 0, 0, new Rotation3d()) };
        GamepieceZone intake = robot.createGamepieceZone("intake", zoneDims, new Rotation3d());
        intake.intake(MetersPerSecond.of(2.0), new Rotation3d());
        StateManager.getInstance().registerGamepieceZone(intake);

        // ── 5. Network Tables publishers ──────────────────────────────────
        WorldPosePublisher worldPub = new WorldPosePublisher(world.getNativeHandle(), 64);
        RobotPosePublisher robotPub = new RobotPosePublisher();
        world.addStepListener(() -> {
            worldPub.publishFrame();
            robotPub.publishFrame();
        });

        // ── 6. Simulation loop (50 Hz, 5 seconds) ─────────────────────────
        for (int i = 0; i < 250; i++) {
            robot.setChassisSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));

            // Single call: dead-reckoning + physics step + zone updates + NT publish
            StateManager.getInstance().stepPhysics();

            Pose2d pose = robot.getPose();
            System.out.printf("t=%.2f  x=%.3f  y=%.3f%n",
                i * 0.02, pose.getX(), pose.getY());
        }

        // ── 7. Cleanup ─────────────────────────────────────────────────────
        worldPub.close();
        robotPub.close();
    }
}
```

For a reference to the `SimRobot` API called inside the loop above, see [`SimRobot`](sim_robot.md).
