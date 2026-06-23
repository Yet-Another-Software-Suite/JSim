# SimBody

`frc.jsim.api.SimBody`

High-level handle to a simulated rigid body. All reads and writes use WPILib geometry types. Obtain instances exclusively through `SimWorld.addBody(SimBodyBuilder)`.

## Identity

| Method | Returns |
|---|---|
| `int getId()` | World-assigned unique ID |
| `String getName()` | Name assigned at creation |
| `RobotId getRobotId()` | Alliance/station identity, or `null` if not a robot body |
| `boolean isRobot()` | `true` if a `RobotId` was set |

## Pose

| Method | Description |
|---|---|
| `Pose3d getPose()` | Current world-frame pose |
| `Translation3d getPosition()` | Current position (metres) |
| `Rotation3d getRotation()` | Current orientation |
| `void setPose(Pose3d)` | Teleport (preserves velocity) |
| `void setPosition(double x, double y, double z)` | Teleport position only |

## Velocity

| Method | Description |
|---|---|
| `Translation3d getLinearVelocity()` | Linear velocity in world frame (m/s) |
| `Translation3d getAngularVelocity()` | Angular velocity in world frame (rad/s) |
| `LinearVelocity getSpeed()` | Scalar speed magnitude |
| `void setLinearVelocity(Translation3d)` | Set from a `Translation3d` |
| `void setLinearVelocity(double vx, double vy, double vz)` | Set from components |
| `void setAngularVelocity(Translation3d)` | Set from a `Translation3d` |
| `void setAngularVelocity(double omX, double omY, double omZ)` | Set from components |

## Physics properties

| Method | Description |
|---|---|
| `Mass getMass()` | Body mass (`POSITIVE_INFINITY` for static) |
| `Material getMaterial()` | Current surface material |
| `void setMaterial(Material)` | Change the surface material |
| `boolean isStatic()` | `true` if the body has infinite mass |

## Actuator

```java
ActuatorForce actuator = body.getActuator();

// Apply a force at the centre of mass (world frame, N):
actuator.setForce(fx, fy, fz);

// Apply a torque (world frame, N·m):
actuator.setTorque(tx, ty, tz);

// Apply a force at an off-centre point (generates torque automatically):
actuator.setForceAtPoint(fx, fy, fz, px, py, pz);

// Clear all inputs (e.g. on disable):
actuator.zero();
```

## Collision

| Method | Description |
|---|---|
| `ColliderShape getCollider()` | The collision shape, or `null` |
| `double[] getAABB()` | World-frame AABB `[minX,minY,minZ,maxX,maxY,maxZ]` |

## See also

- [SimBodyBuilder](sim_body_builder.md)
- [SimWorld](sim_world.md)
- [Javadoc](https://jsim.dev/api/javadocs/frc/jsim/api/SimBody.html)
