3
## JSim Example Robot

This project now runs JSim from the existing robot code. The simulation path is wired through [RobotContainer](src/main/java/frc/robot/RobotContainer.java) and [Robot.java](src/main/java/frc/robot/Robot.java), with a dedicated [JSimShowcase](src/main/java/frc/robot/sim/JSimShowcase.java) helper that exercises the JSim state manager, physics world, telemetry publishing, and built-in gamepiece types.

TODO for JSIM: add `jsim.api.GamepieceZone` to the vendordep so user code can consume it directly instead of carrying a local compatibility shim.
