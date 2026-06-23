# SimWorld

`frc.jsim.api.SimWorld`

Top-level entry point for the JSim physics simulation. Create one per subsystem (or share a singleton), add bodies, and call `step()` each robot loop.

## Constructors

| Signature | Description |
|---|---|
| `SimWorld()` | Default timestep (20 ms / 50 Hz) and 10 solver iterations |
| `SimWorld(double fixedTimestepSeconds)` | Custom timestep |
| `SimWorld(double fixedTimestepSeconds, int solverIterations)` | Custom timestep and iteration count |

## Body management

| Method | Description |
|---|---|
| `SimBody addBody(SimBodyBuilder)` | Build and register a body; returns the handle |
| `void removeBody(SimBody)` | Remove a body and its actuator from the simulation |
| `List<SimBody> getBodies()` | Read-only list of all bodies in insertion order |
| `SimBody findBody(String name)` | First body with the given name, or `null` |
| `SimBody findRobot(RobotId id)` | Body registered with the given alliance/station, or `null` |

## Force generators

| Method | Description |
|---|---|
| `void addForceGenerator(ForceGenerator)` | Register a custom force generator |
| `void removeForceGenerator(ForceGenerator)` | Unregister a force generator |
| `DragForce addDragForce(SimBody, double linear, double angular)` | Create and register velocity-proportional drag |
| `MagnusForce addMagnusForce(SimBody, double k)` | Create and register Magnus lift for spinning bodies |

## Simulation control

| Method | Description |
|---|---|
| `void step()` | Advance by the configured fixed timestep |
| `void step(double dtSeconds)` | Advance by an explicit timestep (breaks determinism if varied) |
| `double getTimestep()` | Returns the configured fixed timestep in seconds |

## Example

```java
SimWorld world = new SimWorld();

world.addBody(new SimBodyBuilder("Floor")
    .planeCollider(0, 0, 1, 0)
    .material(Material.CARPET));

SimBody robot = world.addBody(new SimBodyBuilder("Robot")
    .robotId(RobotId.BLUE_1)
    .mass(54)
    .boxCollider(0.3, 0.3, 0.15)
    .noGravity().fixedRotation()
    .material(Material.CARPET));

// In simulationPeriodic():
world.step();
Pose3d pose = robot.getPose();
```

## See also

- [SimBodyBuilder](sim_body_builder.md)
- [SimBody](sim_body.md)
- [Javadoc](https://jsim.dev/api/javadocs/frc/jsim/api/SimWorld.html)
