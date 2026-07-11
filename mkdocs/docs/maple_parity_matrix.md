# Maple-Sim → JSim Feature Parity Matrix

This matrix tracks Maple-Sim capability mapping and acceptance criteria in JSim.

| Maple-Sim module | JSim module/API | Status | Acceptance criteria |
|---|---|---|---|
| Simulated arena lifecycle | `jsim.simulation.SimulatedArena` | Implemented | Arena can load season pack, tick lifecycle through a single `simulationPeriodic()` call, and expose field + battery handles. |
| Projectile model | `jsim.simulation.GamePieceProjectile` | Implemented | Projectile can spawn from builder, update each tick, and auto-despawn on completion predicate, timeout, or below-floor condition. |
| Intake abstraction | `jsim.simulation.IntakeSimulation` | Implemented | Intake can acquire nearest valid game piece within radius and eject with configured velocity. |
| Opponent abstraction | `jsim.simulation.drivesims.OpponentSimulation` | Implemented | Opponent body can be driven via WPILib `ChassisSpeeds`/`Pose2d` suppliers and updated during arena lifecycle. |
| Drivetrain simulation hook | `jsim.simulation.drivesims.DriveTrainSimulation` | Implemented | Robot-relative chassis speeds are converted to world-frame body velocities every arena tick. |
| Motor simulation | `jsim.simulation.motorsims.MotorSimulation` | Implemented | DC motor state updates deterministic position/velocity/current from voltage + load torque + dt. |
| Battery simulation | `jsim.simulation.motorsims.SimulatedBattery` | Implemented | Battery aggregates current draw, computes loaded voltage, and publishes to WPILib `RoboRioSim`. |
| Season-specific packs | `jsim.simulation.SeasonConfig` + `SeasonRegistry` | Implemented | Season packs resolve by canonical key or alias, include explicit version, and support migration-safe lookup. |
| Docs/examples/tests | docs + unit tests | Implemented | Parity table is published and new simulation APIs are covered by targeted unit tests. |

## Notes

- Parity here focuses on API-level and lifecycle capability in JSim.
- Additional season packs can be added with `SeasonRegistry.register(...)`.
