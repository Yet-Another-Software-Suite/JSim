# PhysicsWorld, PhysicsBody, and Gamepiece

API reference for the three core physics-simulation types in JSim.

---

## PhysicsWorld

`PhysicsWorld` is a thin Java wrapper around the native 3D physics engine. It owns all rigid bodies and gamepieces, advances the simulation clock, and exposes hooks for per-step callbacks.

### Creation

```java
// Preferred — uses JSim factory
PhysicsWorld world = JSim.createPhysicsWorld(0.005, true); // 5 ms fixed dt, gravity on

// Direct constructor
PhysicsWorld world = new PhysicsWorld(double fixedDtSeconds, boolean enableGravity);
```

!!! note
    After creating the world, register it with `StateManager` so that robot collision bodies are automatically created and updated each tick.

    ```java
    StateManager.getInstance().setPhysicsWorld(world);
    ```

---

### Configuration methods

| Method | Description |
|--------|-------------|
| `setGravity(double gx, double gy, double gz)` | Set gravity vector in m/s² along each axis |
| `setGravity(Translation3d gravity)` | Set gravity vector using a WPILib `Translation3d` |

---

### Body and gamepiece creation

| Method | Return type | Description |
|--------|-------------|-------------|
| `createBody(double massKg)` | `PhysicsBody` | Allocate a new free rigid body with the given mass |
| `createGamepiece()` | `Gamepiece` | Create a default-configured gamepiece |
| `createGamepiece(double radius, double mass, double restitution)` | `Gamepiece` | Create a spherical gamepiece with explicit physics parameters |
| `createGamepiece(GamePieceType type)` | `Gamepiece` | Create a gamepiece matching a named `GamePieceType` preset |
| `createGamepiece(GamePieceType type, double radius, double mass, double restitution)` | `Gamepiece` | Create a typed gamepiece with overridden physics parameters |
| `gamepieces()` | `List<Gamepiece>` | Return the live list of all gamepieces owned by this world |

---

### Simulation control

| Method | Return type | Description |
|--------|-------------|-------------|
| `step()` | `void` | Advance the simulation by exactly one fixed timestep |
| `step(int steps)` | `void` | Advance the simulation by N fixed timesteps |
| `addStepListener(Runnable listener)` | `void` | Register a callback invoked after every `step()` call |
| `getFixedDtSeconds()` | `double` | Return the fixed timestep duration in seconds |
| `getNativeHandle()` | `long` | Return the native engine handle (used by NetworkTables publishers) |
| `getMaxBodies()` | `int` | Return the maximum number of bodies supported (default: 128) |

---

### Gamepiece state methods

These low-level methods operate on gamepieces by index and are primarily used internally or by NetworkTables publishers. Prefer the `Gamepiece` handle API for user code.

| Method | Description |
|--------|-------------|
| `getGamepiecePosition(int index)` | Return the current `Pose3d` of the gamepiece at the given index |
| `setGamepiecePosition(int index, double x, double y, double z)` | Teleport a gamepiece to the given position |
| `outtakeGamepiece(int index, double x, double y, double z, double vx, double vy, double vz)` | Reposition a gamepiece and assign it an initial velocity |

---

## PhysicsBody

`PhysicsBody` is a handle for a single rigid body managed by `PhysicsWorld`. When `PhysicsWorld` is registered with `StateManager`, a `PhysicsBody` is auto-created for each simulated robot and kept in sync with its pose every tick.

!!! tip
    You do not normally need to create `PhysicsBody` instances by hand. `StateManager` handles body creation after you call `setPhysicsWorld(world)`.

---

### Setters

| Method | Description |
|--------|-------------|
| `setPosition(Pose3d pose)` | Teleport the body to the given pose |
| `setLinearVelocity(double vx, double vy, double vz)` | Set the body's linear velocity in m/s |
| `setOrientation(Rotation3d rotation)` | Set the body's orientation without changing its translation |
| `setGravityEnabled(boolean enabled)` | Enable or disable gravity for this body individually |
| `setCollisionBox(double x, double y, double z)` | Set an axis-aligned box collider with the given half-extents in meters |
| `setCollisionSphere(double radiusMeters)` | Set a sphere collider with the given radius |
| `setCollisionFilter(int layerBits, int maskBits)` | Configure which collision layers this body belongs to and which it collides with |

---

### Getters

| Method | Return type | Description |
|--------|-------------|-------------|
| `position()` | `Pose3d` | Current pose of the body |
| `linearVelocity()` | `LinearVelocity3d` | Current linear velocity of the body |
| `orientation()` | `Rotation3d` | Current orientation of the body |
| `bodyIndex()` | `int` | Internal index used by the native engine |

---

## Gamepiece

`Gamepiece` is a handle for a single physics-simulated game element — a ball, fuel cell, coral, or custom type. Gamepieces are created via `PhysicsWorld` and interact with `GamepieceZone` for automated intake/outtake behavior.

---

### Position and velocity

| Method | Description |
|--------|-------------|
| `setPosition(Translation3d pos)` | Teleport the gamepiece to the given field position |
| `setLinearVelocity(double vx, double vy, double vz)` | Set the gamepiece's linear velocity in m/s |

---

### Interaction

| Method | Return type | Description |
|--------|-------------|-------------|
| `pick(Pose3d intakePos, double captureRadius, Vec3 carryOffset)` | `boolean` | Attempt to capture the gamepiece. Returns `true` if the gamepiece was within `captureRadius` meters of `intakePos` and is now being carried at `carryOffset` relative to the robot |
| `place(Translation3d pos)` | `void` | Drop the gamepiece at the given field position with zero velocity |
| `outtake(Pose3d pose, LinearVelocity3d vel)` | `void` | Release the gamepiece at `pose` with the given initial velocity |

---

### Getters

| Method | Return type | Description |
|--------|-------------|-------------|
| `position()` | `Pose3d` | Current pose of the gamepiece |
| `linearVelocity()` | `LinearVelocity3d` | Current linear velocity of the gamepiece |
| `type()` | `GamePieceType` | The `GamePieceType` enum value, or `null` for untyped gamepieces |
| `typeName()` | `String` | The string name of the type, or `null` for untyped gamepieces |
| `gamepieceIndex()` | `int` | Internal index used by the native engine |
| `isBall()` | `boolean` | Convenience check — returns `true` when `type()` is `GamePieceType.BALL` |

---

### Example: spawn, position, and outtake a gamepiece

```java
PhysicsWorld world = JSim.createPhysicsWorld(0.005, true);
StateManager.getInstance().setPhysicsWorld(world);

// Create a ball gamepiece
Gamepiece ball = world.createGamepiece(GamePieceType.BALL);

// Place it at the center of the field, on the ground
ball.setPosition(new Translation3d(8.27, 4.10, 0.12));

// Later — outtake the ball from the shooter pose at 10 m/s upward
Pose3d shooterPose = new Pose3d(robot.getX(), robot.getY(), 0.9,
        new Rotation3d(0, Math.toRadians(-45), robot.getHeadingRad()));
LinearVelocity3d exitVel = new LinearVelocity3d(0, 0, 10.0); // m/s

ball.outtake(shooterPose, exitVel);
```

!!! note
    After `outtake()` the gamepiece re-enters free flight under gravity. Call `world.step()` each robot loop iteration to advance its trajectory.

---

### GamePieceType enum values

| Value | Description |
|-------|-------------|
| `BALL` | Standard spherical ball (e.g. Power Cell, Note) |
| `FUEL` | Fuel-type gamepiece |
| `CORAL` | Coral-type gamepiece (2025 season) |
| `CUSTOM1` | User-defined type 1 |
| `CUSTOM2` | User-defined type 2 |
| `CUSTOM3` | User-defined type 3 |
| `CUSTOM4` | User-defined type 4 |
