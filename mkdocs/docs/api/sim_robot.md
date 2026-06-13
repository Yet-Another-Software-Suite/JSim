# SimRobot

`SimRobot` is the external handle for a simulated FRC robot inside JSim. Each instance is registered with and owned by [`StateManager`](state_manager.md); you do not construct it directly — you get one back from `SimRobot.createRobot()` or `StateManager.getInstance().initializeRobot()`.

A `SimRobot` carries:

- A field-relative **pose** (odometry + physics-body read-back)
- A **chassis speeds** command that the physics solver consumes
- A convex **frame footprint** used to size the collision box
- An optional set of named **gamepiece zones** for intake / outtake / shoot logic

!!! note "Package"
    `jsim.api.SimRobot`

---

## RobotID

Every robot slot on the field is identified by a `RobotID` enum value. Pass one of these to every `SimRobot` / `StateManager` call that targets a specific robot.

| Constant | Alliance | Driver station |
|----------|----------|----------------|
| `RobotID.RED_1` | Red | 1 |
| `RobotID.RED_2` | Red | 2 |
| `RobotID.RED_3` | Red | 3 |
| `RobotID.BLUE_1` | Blue | 1 |
| `RobotID.BLUE_2` | Blue | 2 |
| `RobotID.BLUE_3` | Blue | 3 |

```java
import jsim.api.RobotID;

RobotID id = RobotID.BLUE_1;
```

---

## Creating a robot

### `SimRobot.createRobot(RobotID, Translation2d[])`

Static factory. Delegates to `StateManager.getInstance().initializeRobot()` with a zero starting pose and registers the robot in the simulation.

```java
import jsim.api.SimRobot;
import jsim.api.RobotID;
import edu.wpi.first.math.geometry.Translation2d;

Translation2d[] frame = {
    new Translation2d( 0.381,  0.381),
    new Translation2d(-0.381,  0.381),
    new Translation2d(-0.381, -0.381),
    new Translation2d( 0.381, -0.381)
};

SimRobot robot = SimRobot.createRobot(RobotID.BLUE_1, frame);
```

!!! warning "Call order"
    Call `StateManager.getInstance().setPhysicsWorld()` **before** `createRobot()`. Robots created before a physics world is registered will not have a valid physics body. See [`StateManager`](state_manager.md#physics-world).

### `SimRobot.getRobot(RobotID)`

Static lookup. Returns the previously registered robot, or `null` if none exists for that `RobotID`.

```java
SimRobot robot = SimRobot.getRobot(RobotID.BLUE_1);
if (robot != null) {
    Pose2d pose = robot.getPose();
}
```

---

## Pose methods

| Method | Returns | Description |
|--------|---------|-------------|
| `getPose()` | `Pose2d` | Physics body position when a world is attached; falls back to the last commanded (odometry) pose when no physics body is registered |
| `getOdometryPose()` | `Pose2d` | Always the last commanded pose — unaffected by collisions or physics disturbances |
| `resetPose(Pose2d)` | `void` | Hard-resets the simulation pose; teleports the robot to an arbitrary location immediately |

!!! note "Which pose to use?"
    Use `getPose()` for visualization and anything that should reflect real collision results. Use `getOdometryPose()` when testing odometry algorithms in isolation without physics disturbances.

```java
Pose2d physicsPose  = robot.getPose();
Pose2d odometryPose = robot.getOdometryPose();

// Teleport to autonomous starting position
robot.resetPose(new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(0)));
```

---

## Speed methods

| Method | Description |
|--------|-------------|
| `setChassisSpeeds(ChassisSpeeds)` | Feeds commanded speeds into JSim; the physics solver reads these each tick via `StateManager.stepPhysics()` |
| `getVelocity()` | Returns the physics body's measured linear velocity when a body is attached; falls back to the last commanded `ChassisSpeeds` otherwise |

```java
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// Drive forward at 1 m/s
robot.setChassisSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));
StateManager.getInstance().stepPhysics();

ChassisSpeeds measured = robot.getVelocity();
```

!!! tip "Closed-loop control"
    Always call `setChassisSpeeds()` before `stepPhysics()` each iteration so the physics engine uses the current cycle's command.

---

## Frame vertices

`robot.getFrameVertices()` returns a `Translation2d[]` array — the corners of the robot's convex hull in **robot-relative coordinates** (meters, origin at robot center).

These vertices are passed in at construction time and are used by `StateManager` to size the axis-aligned collision box when registering the robot body with the physics engine. A 76 cm × 76 cm square robot would look like:

```java
Translation2d[] frame = {
    new Translation2d( 0.381,  0.381),   // front-left
    new Translation2d(-0.381,  0.381),   // rear-left
    new Translation2d(-0.381, -0.381),   // rear-right
    new Translation2d( 0.381, -0.381)    // front-right
};
```

The physics engine derives `width = maxX − minX` and `depth = maxY − minY` from these points and creates a box collider of that size. If no vertices are supplied a 0.7 m × 0.7 m default is used.

---

## Gamepiece zones

A `GamepieceZone` is a named 3-D polygon attached to the robot that can capture or eject gamepieces. Zones are robot-relative and rotate with the robot each tick.

Create a zone with:

```java
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;

Transform3d[] dims = { new Transform3d(0.3, 0, 0, new Rotation3d()) };
GamepieceZone intake = robot.createGamepieceZone("intake", dims, new Rotation3d());
```

Then register it so `StateManager` refreshes it each tick:

```java
StateManager.getInstance().registerGamepieceZone(intake);
```

| Method | Returns | Description |
|--------|---------|-------------|
| `createGamepieceZone(String, Transform3d[], Rotation3d)` | `GamepieceZone` | Creates and stores a named zone on this robot |
| `getGamepieceZone(String)` | `GamepieceZone` or `null` | Retrieves a previously created zone by name |
| `getGamepieceZones()` | `Map<String, GamepieceZone>` (immutable) | All zones registered on this robot |

For full zone API details — modes (`INTAKE`, `OUTTAKE`, `SHOOT`, `DISABLED`), supplier-based dynamic configuration, and exit velocity / rotation — see [GamepieceZone](gamepiece_zone.md).

---

## Inner class: `RobotState`

`SimRobot.RobotState` is the mutable state record that `StateManager` stores internally for each robot. You do not access it directly, but its fields are exposed through the `SimRobot` accessor methods above.

| Field | Type | Description |
|-------|------|-------------|
| `robotPose` | `Pose2d` | Current field-relative pose (odometry) |
| `frameVertices` | `Translation2d[]` | Frame perimeter corners in robot-relative coordinates |
| `speeds` | `ChassisSpeeds` | Last commanded chassis speeds |

---

## `getRobotID()`

Returns the `RobotID` assigned to this robot at construction.

```java
RobotID id = robot.getRobotID(); // e.g. RobotID.BLUE_1
```

---

## `update(double dtSeconds)`

!!! warning "Internal — do not call directly"
    `update()` is called by `StateManager.stepPhysics()` automatically each tick. Calling it manually will cause double-integration and incorrect pose results.

Advances the robot's dead-reckoning pose by one timestep using the current `ChassisSpeeds`.

---

## Minimal setup example

```java
import jsim.JSim;
import jsim.PhysicsWorld;
import jsim.api.StateManager;
import jsim.api.SimRobot;
import jsim.api.RobotID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MinimalRobotSetup {

    public static void main(String[] args) {

        // 1. Physics world — must come before createRobot()
        PhysicsWorld world = JSim.createPhysicsWorld(0.02, false);
        StateManager.getInstance().setPhysicsWorld(world);

        // 2. Define frame footprint (76 cm square, robot-relative)
        Translation2d[] frame = {
            new Translation2d( 0.381,  0.381),
            new Translation2d(-0.381,  0.381),
            new Translation2d(-0.381, -0.381),
            new Translation2d( 0.381, -0.381)
        };

        // 3. Create the robot and set its starting pose
        SimRobot robot = SimRobot.createRobot(RobotID.BLUE_1, frame);
        robot.resetPose(new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(0)));

        // 4. Simulation loop (50 Hz, 3 seconds)
        for (int i = 0; i < 150; i++) {
            robot.setChassisSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));
            StateManager.getInstance().stepPhysics();

            Pose2d pose = robot.getPose();
            System.out.printf("t=%.2f  x=%.3f  y=%.3f%n",
                i * 0.02, pose.getX(), pose.getY());
        }
    }
}
```

For a complete example including gamepiece zones and Network Tables publishing see [API Usage](../api_usage.md).
