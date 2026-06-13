# Swerve Drive Walkthrough (YAMS + JSim)

This walkthrough covers how to wire JSim into a swerve drive subsystem built on
[YAMS](https://github.com/BroncBotz3481/YAMS) (`SwerveDrive`). By the end you
will have physics-based pose estimation running in simulation instead of the
pure-odometry estimate that `SwerveDrive` produces on its own.

The complete reference implementation lives at
`examples/java/simple_robot/src/main/java/frc/robot/subsystems/SwerveSubsystem.java`.

---

## Overview

Without JSim, `SwerveDrive` tracks the robot's position by integrating wheel
encoder and gyro readings (odometry). Odometry has no concept of inertia,
friction, collisions, or any other real-world physics — the simulated robot
teleports instantly to wherever the commanded speeds say it should be.

JSim adds a physics body to the robot. Each simulation tick you hand JSim the
current chassis speeds; JSim integrates those speeds through its physics engine
and returns a `Pose2d` that reflects realistic motion (acceleration curves,
momentum, and — if a `PhysicsWorld` is registered — full collision response).
You then push that pose back into YAMS so every subsystem, path-follower, and
telemetry consumer sees the physics-corrected position.

!!! note
    JSim's physics simulation runs on the roboRIO simulation thread inside
    `simulationPeriodic()`. It does **not** run on real hardware; your
    `periodic()` method is unaffected.

---

## Step 1: Define frame vertices

Frame vertices describe the robot's physical footprint as a convex polygon in
the robot's local coordinate frame (X forward, Y left, origin at the center of
the robot). JSim uses this polygon for collision detection and to compute the
moment of inertia of the chassis.

Each vertex is a `Translation2d` measured **from the robot center** in meters.
For a square robot whose swerve modules sit at ±24 inches from center, the four
corners are:

```java
Translation2d[] frameVertices = {
    new Translation2d(Inches.of(24).in(Meters),  Inches.of(24).in(Meters)),   // front-left
    new Translation2d(Inches.of(24).in(Meters),  Inches.of(-24).in(Meters)),  // front-right
    new Translation2d(Inches.of(-24).in(Meters), Inches.of(-24).in(Meters)),  // back-right
    new Translation2d(Inches.of(-24).in(Meters), Inches.of(24).in(Meters))    // back-left
};
```

The vertices do not have to match the module locations exactly — they define the
**chassis boundary**, not the module mounting points. List them in order
(clockwise or counter-clockwise) so the polygon is convex.

!!! tip
    `Inches.of(24).in(Meters)` evaluates to `0.6096`. You can use the literal
    directly if you prefer, but the WPILib units expression is self-documenting
    and avoids unit conversion mistakes.

---

## Step 2: Create the SimRobot in the constructor

After building the `SwerveDrive` object, create a `SimRobot` and set its
starting pose:

```java
simRobot = SimRobot.createRobot(RobotID.BLUE_1, frameVertices);
simRobot.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
```

### RobotID

`RobotID` identifies which robot on the field this physics body represents.
Use `RobotID.BLUE_1` for the primary (or only) simulated robot. If you are
simulating multiple robots simultaneously — for example, in a multi-agent test —
each must have a **unique** `RobotID` (e.g. `BLUE_2`, `RED_1`, etc.).

### resetPose

`resetPose` sets the initial position of the physics body. Pass the same
starting pose you gave to `SwerveDriveConfig.withStartingPose()` so the JSim
body and the YAMS odometry start in agreement.

### Declare the field

Also register a `Field2d` with SmartDashboard so you can visualize the robot
pose in the simulator dashboard:

```java
private final Field2d field = new Field2d();
// ...inside the constructor:
SmartDashboard.putData("Field", field);
```

---

## Step 3: Wire simulationPeriodic()

Override `simulationPeriodic()` in your subsystem and add the following six
lines in order:

```java
@Override
public void simulationPeriodic()
{
    drive.simIterate();                                       // (1)
    simRobot.setChassisSpeeds(drive.getRobotRelativeSpeed()); // (2)
    StateManager.getInstance().stepPhysics();                 // (3)
    Pose2d jsimPose = simRobot.getPose();                     // (4)
    drive.resetOdometry(jsimPose);                            // (5)
    field.setRobotPose(jsimPose);                             // (6)
}
```

### What each line does

| Line | Call | Purpose |
|------|------|---------|
| 1 | `drive.simIterate()` | Advances the YAMS motor simulation models (drive and azimuth `SparkWrapper`s, etc.) by one 20 ms tick so that `getRobotRelativeSpeed()` reflects the motor physics. |
| 2 | `simRobot.setChassisSpeeds(...)` | Passes the current robot-relative chassis speeds into JSim. This is the velocity input the physics integrator will use this tick. |
| 3 | `StateManager.getInstance().stepPhysics()` | Advances the JSim physics engine by one simulation step, integrating the speeds set in line 2. |
| 4 | `simRobot.getPose()` | Reads back the physics-aware pose. If a `PhysicsWorld` is registered this includes collision response; otherwise it is a high-fidelity integration of the chassis speeds. |
| 5 | `drive.resetOdometry(jsimPose)` | Writes the JSim pose back into YAMS so that `drive.getPose()` — and every consumer that calls it — sees the physics-corrected position. |
| 6 | `field.setRobotPose(jsimPose)` | Updates the subsystem's own `Field2d` (published as `"Field"` on SmartDashboard) with the JSim pose. |

!!! warning
    Always call `drive.simIterate()` **before** `setChassisSpeeds`. YAMS
    updates its internal speed estimate during `simIterate()`; reading
    `getRobotRelativeSpeed()` before that call returns a one-step-stale value.

---

## How the Field2d update propagates

There are two separate `Field2d` objects in play:

1. **YAMS internal `m_field2d`** — owned by `SwerveDrive`, published under the
   key `"Mechanisms/swerve/field"`. YAMS has no public getter for it; instead,
   the next `periodic()` call to `drive.updateTelemetry()` runs
   `m_field2d.setRobotPose(getPose())`. Because line 5 above called
   `drive.resetOdometry(jsimPose)`, `drive.getPose()` now returns the JSim pose,
   so the YAMS telemetry widget automatically reflects the physics simulation.

2. **Subsystem `field`** — the `Field2d` you declared and registered with
   `SmartDashboard.putData("Field", field)`. Line 6 updates this directly from
   `jsimPose` every simulation tick.

Both displays stay in sync without any extra bookkeeping.

---

## Full imports

Add these three JSim imports alongside the standard WPILib imports:

```java
import jsim.api.RobotID;
import jsim.api.SimRobot;
import jsim.api.StateManager;
```

The complete import block for the subsystem also includes WPILib geometry and
kinematics classes (`Pose2d`, `Rotation2d`, `Translation2d`, `ChassisSpeeds`,
`Field2d`) which you likely already have.

---

## Tips

!!! tip "Commanded pose vs. physics pose"
    `simRobot.getPose()` returns the **physics-aware** position. If you need
    the pose that would result from pure dead-reckoning (no collision response,
    no inertia model) use `simRobot.getOdometryPose()` instead. This is useful
    for debugging: if the two diverge significantly, a collision or large
    impulsive force occurred.

!!! tip "Enabling full collision support"
    By default JSim integrates chassis speeds without obstacle collision. To
    enable field-element and robot-to-robot collisions, create a `PhysicsWorld`
    and register it before the first `stepPhysics()` call:
    ```java
    StateManager.setPhysicsWorld(world);
    ```
    Once a `PhysicsWorld` is registered, `simRobot.getPose()` returns the
    position of the rigid body inside that world, with full collision impulse
    response.

!!! warning "Unique RobotID per robot"
    Each `SimRobot` on the field must be created with a distinct `RobotID`.
    Reusing the same `RobotID` for two robots causes the second `createRobot`
    call to overwrite the first body, leading to unpredictable simulation
    results.
