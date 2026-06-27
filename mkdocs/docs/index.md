# JSim

**Deterministic 3D physics simulation for FRC robots**

JSim gives your robot code a physics world it can step through each control loop iteration. Bodies collide, bounce, and respond to motor forces using impulse-based rigid-body dynamics, all from a clean Java API built on WPILib geometry types.

---

## What JSim does

- **Rigid-body physics** — spheres, boxes, and infinite planes with realistic collision response (Baumgarte + Sequential Impulse solver)
- **Force generators** — built-in gravity, linear drag, Magnus lift for spinning game pieces, and per-body actuator inputs from motor outputs
- **WPILib-native API** — all poses and velocities use `Pose3d`, `Translation3d`, `Rotation3d`, and WPILib units
- **Deterministic** — fixed 20 ms timestep, raw-double internal state; two runs with the same inputs always produce the same results
- **Self-contained example** — a complete swerve-drive robot in `examples/java/simple_robot/` that you can compile and simulate out of the box

---

## Quick start

### 1. Add the vendordep

In VS Code WPILib or from the command line, install from:

```
https://jsim.dev/JSim.json
```

Or add the JSON URL to your `.wpilib/vendordeps/` folder manually. See [Getting Started](getting_started.md) for details.

### 2. Create a world

```java
// In your subsystem or Robot class:
SimWorld simWorld = new SimWorld();          // 50 Hz fixed timestep

// Static floor (infinite half-space facing +Z)
simWorld.addBody(new SimBodyBuilder("Floor")
    .planeCollider(0, 0, 1, 0)
    .material(Material.CARPET));

// Robot body — tagged as Blue alliance station 1
SimBody robot = simWorld.addBody(new SimBodyBuilder("Robot")
    .robotId(RobotId.BLUE_1)
    .position(0, 0, 0.1)
    .mass(54)                                // kg
    .boxCollider(0.3, 0.3, 0.15)            // half-extents in metres
    .noGravity()
    .fixedRotation()
    .material(Material.CARPET));
```

### 3. Step each loop

```java
@Override
public void simulationPeriodic() {
    simWorld.step();                         // advance by 20 ms
    Pose3d pose = robot.getPose();
    field.setRobotPose(pose.toPose2d());
}
```

---

## Links

| Resource | URL |
|---|---|
| Javadoc | [jsim.dev/api/javadocs](https://jsim.dev/api/javadocs) |
| GitHub | [github.com/Yet-Another-Software-Suite/JSim](https://github.com/Yet-Another-Software-Suite/JSim) |
| Maven | [jsim.dev/maven/releases](https://jsim.dev/maven/releases/) |
| Vendordep JSON | [jsim.dev/JSim.json](https://jsim.dev/JSim.json) |

---

## Example project

The `examples/java/simple_robot/` directory is a complete, buildable FRC swerve-drive robot that integrates JSim for desktop simulation. Open it directly in VS Code with the WPILib extension.

See [Swerve Drive Simulation](walkthroughs/swerve_drive.md) for a guided walkthrough.
