# NetworkTables Publishing

JSim publishes simulation state to NetworkTables so you can visualize it in AdvantageScope or any NT-aware dashboard. The `jsim.nt` package provides two publisher classes for this purpose:

- `WorldPosePublisher` — publishes rigid body positions and velocities for every body in the physics world
- `RobotPosePublisher` — publishes robot poses specifically, including a `field2d`-compatible format

Both classes implement `AutoCloseable` and are designed to be wired as step listeners on `PhysicsWorld`.

---

## WorldPosePublisher

`jsim.nt.WorldPosePublisher`

Publishes the full state of every body tracked by the physics world. On each call to `publishFrame()`, it reads the current body states from the native physics engine and writes them to NetworkTables.

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/JSim/World/bodyPoses` | `Pose3d[]` | Current pose of each body |
| `/JSim/World/bodyState13Flat` | `double[]` | Flat state per body: `[x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz, ...]` |
| `/JSim/World/bodyPose7Flat` | `double[]` | Flat pose per body: `[x, y, z, qw, qx, qy, qz, ...]` |
| `/JSim/World/bodyVelocity6Flat` | `double[]` | Flat velocity per body: `[vx, vy, vz, wx, wy, wz, ...]` |

### Constructors

```java
// Uses the default NetworkTableInstance and /JSim/World base topic
new WorldPosePublisher(long worldHandle, int maxBodies)

// Specify a custom NT instance and base topic path
new WorldPosePublisher(long worldHandle, int maxBodies, NetworkTableInstance nt, String baseTopic)
```

| Parameter | Description |
|-----------|-------------|
| `worldHandle` | Native handle obtained from `PhysicsWorld.getNativeHandle()` |
| `maxBodies` | Maximum number of bodies to publish; use `PhysicsWorld.getMaxBodies()` (returns `128`) |
| `nt` | NT instance to publish to |
| `baseTopic` | Root NT path for all published topics |

### Methods

**`int publishFrame()`**

Reads the current physics state and publishes one frame to NetworkTables. Returns the number of bodies published.

**`void close()`**

Releases all NT publishers. Call this when tearing down the simulation or when the object is no longer needed.

---

## RobotPosePublisher

`jsim.nt.RobotPosePublisher`

Publishes robot poses sourced from `StateManager`. On each call to `publishFrame()`, it reads all robot poses registered with the `StateManager` and writes them to NetworkTables, including a `field2d`-compatible array.

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/JSim/RobotPose/robotIds` | `String[]` | Robot identifier names |
| `/JSim/RobotPose/robotPoses` | `Pose3d[]` | 3D poses of all robots |
| `/JSim/RobotPose/robotPose3Flat` | `double[]` | Flat `[x, y, theta, ...]` per robot |
| `/field2d/JSim` | `Pose2d[]` | AdvantageScope-compatible field2d array |

### Constructors

```java
// Uses the default NetworkTableInstance and /JSim/RobotPose base topic
new RobotPosePublisher()

// Specify a custom NT instance and base topic path
new RobotPosePublisher(NetworkTableInstance nt, String baseTopic)
```

| Parameter | Description |
|-----------|-------------|
| `nt` | NT instance to publish to |
| `baseTopic` | Root NT path for all published topics |

### Methods

**`int publishFrame()`**

Reads robot poses from `StateManager` and publishes one frame to NetworkTables. Returns the number of robots published.

**`void close()`**

Releases all NT publishers. Call this when tearing down the simulation or when the object is no longer needed.

---

## Wiring Example

Wire both publishers as step listeners on `PhysicsWorld` so they run automatically after every physics step.

```java
@Override
public void robotInit() {
    PhysicsWorld world = JSim.createPhysicsWorld(0.02, false);
    StateManager.getInstance().setPhysicsWorld(world);

    WorldPosePublisher worldPub = new WorldPosePublisher(world.getNativeHandle(), world.getMaxBodies());
    RobotPosePublisher robotPub = new RobotPosePublisher();

    // Publishers run automatically after every physics step
    world.addStepListener(() -> {
        worldPub.publishFrame();
        robotPub.publishFrame();
    });
}
```

!!! tip
    Both publishers are `AutoCloseable`. If you manage them in a try-with-resources block or register them with a resource manager, `close()` will be called automatically on shutdown.

!!! note
    `RobotPosePublisher` reads from `StateManager`, so `StateManager.getInstance().setPhysicsWorld(world)` must be called before the first `publishFrame()`.

---

## AdvantageScope Setup

### 3D Field Visualization

1. Open AdvantageScope and navigate to the **3D Field** tab.
2. Add a source for `/JSim/RobotPose/robotPoses` (type `Pose3d[]`) to visualize robot positions.
3. Add a source for `/JSim/World/bodyPoses` (type `Pose3d[]`) to visualize gamepieces and other bodies.

### 2D Field Visualization

1. Navigate to the **2D Field** tab.
2. Add `/field2d/JSim` (type `Pose2d[]`) as the robot pose source.

!!! tip
    The `/field2d/JSim` topic uses the standard WPILib `field2d` format, so it works with any dashboard that supports that protocol — not just AdvantageScope.

---

## Custom NT Base Topic

If you need to publish to a different NT path (for example, to avoid conflicts with another simulator instance), pass a custom base topic to the constructors:

```java
WorldPosePublisher worldPub = new WorldPosePublisher(
    world.getNativeHandle(),
    128,
    NetworkTableInstance.getDefault(),
    "/MyRobot/Sim"
);

RobotPosePublisher robotPub = new RobotPosePublisher(
    NetworkTableInstance.getDefault(),
    "/MyRobot/Sim"
);
```

!!! warning
    When using a custom base topic, the `field2d`-compatible topic path will also change relative to the new base. Update your dashboard sources accordingly.
