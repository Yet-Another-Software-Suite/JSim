# Architecture

## Package layout

```
frc.jsim.api/          Public API — everything robot code needs
  SimWorld             Top-level entry point
  SimBody              Body handle (WPILib types at the surface)
  SimBodyBuilder       Fluent builder
  BodyTracker          History and distance utility
  RobotId              Alliance/station identity

frc.jsim.core/         Physics engine internals
  PhysicsWorld         Simulation loop owner
  SimConstants         Global constants (dt, gravity, solver params)
  Vec3                 Raw-double 3-vector math
  Mat3                 Raw-double 3×3 matrix math

frc.jsim.dynamics/     Rigid body state
  RigidBody            All physics state as raw doubles
  RigidBodyFlags       Bitmask constants (STATIC, NO_GRAVITY, …)

frc.jsim.collision/    Collision detection
  ColliderShape        Abstract shape base
  SphereCollider       Sphere by radius
  BoxCollider          Box by half-extents
  PlaneCollider        Infinite half-space
  BroadPhase           O(n²) AABB overlap
  NarrowPhase          Geometry-accurate contact generation
  ContactPoint         Single contact record

frc.jsim.solver/       Constraint solving
  ImpulseSolver        Sequential Impulse (PGS)
  ContactConstraint    Pre-computed constraint cache

frc.jsim.forces/       Force generators
  ForceGenerator       Interface (functional)
  GravityForce         Uniform gravity
  ActuatorForce        Motor / pneumatic input
  DragForce            Velocity-proportional damping
  MagnusForce          Spinning-projectile lift

frc.jsim.material/     Surface properties
  Material             friction + restitution
  MaterialCombiner     Contact material combination rules
```

---

## Deterministic tick pipeline

Each call to `SimWorld.step()` executes exactly this sequence:

1. **Clear accumulators** — zero force and torque on every body
2. **Apply forces** — gravity, actuators, drag, magnus (in registration order)
3. **Broadphase** — AABB overlap; O(n²), fine for ≤50 bodies
4. **Narrowphase** — shape-exact contact generation (sphere/box/plane pairs)
5. **Solve** — Sequential Impulse PGS (10 iterations); resolves normal + friction constraints with Baumgarte positional correction baked into the bias term
6. **Integrate** — semi-implicit Euler: `v += a·dt`, `x += v·dt`; quaternion update
7. **Derive** — AABB refresh, quaternion normalization, world-frame inertia refresh

## Determinism guarantees

All physics state is stored as raw `double` primitives in `RigidBody`. The fixed 20 ms timestep ensures the same sequence of floating-point operations across runs. WPILib geometry types are only used at the API boundary (setPose, getPose, etc.) and do not affect the integration path.
