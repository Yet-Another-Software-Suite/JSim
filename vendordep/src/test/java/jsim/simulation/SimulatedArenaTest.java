package jsim.simulation;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import jsim.api.SimBody;
import jsim.api.SimBodyBuilder;
import jsim.material.Material;
import org.junit.jupiter.api.Test;

class SimulatedArenaTest {
    @Test
    void driveTrainHookUpdatesRobotVelocity() {
        SimulatedArena arena = SimulatedArena.fromSeason("reefscape2025");
        SimBody robot = arena.addRobot(new SimBodyBuilder("Robot")
                .position(2, 2, 0.1)
                .mass(54)
                .boxCollider(0.3, 0.3, 0.1)
                .noGravity()
                .fixedRotation()
                .material(Material.CARPET));
        arena.registerDriveTrain(robot, () -> new ChassisSpeeds(2.0, 0.0, 0.5), Pose2d::new);

        arena.simulationPeriodic();

        assertTrue(robot.getLinearVelocity().getX() > 1.5);
        assertTrue(robot.getAngularVelocity().getZ() > 0.4);
    }

    @Test
    void intakeAndProjectileLifecycleAreManagedByArena() {
        SimulatedArena arena = SimulatedArena.fromSeason("reefscape2025");
        SimBody robot = arena.addRobot(new SimBodyBuilder("Robot")
                .position(4, 4, 0.1)
                .mass(54)
                .boxCollider(0.3, 0.3, 0.1)
                .noGravity()
                .fixedRotation()
                .material(Material.CARPET));
        IntakeSimulation intake = arena.createIntake(robot, new Translation3d(0.4, 0, 0), 0.7);
        arena.spawnGamePiece("Coral", new SimBodyBuilder("Coral0")
                .position(4.35, 4.0, 0.1)
                .mass(0.25)
                .sphereCollider(0.08)
                .material(Material.RUBBER));

        boolean grabbed = intake.intakeNearest(piece -> true);
        assertTrue(grabbed);
        assertTrue(intake.isHolding());

        int before = arena.getFieldSimulator().getGamePieces().size();
        arena.launchProjectile("Coral", new SimBodyBuilder("Shot")
                .position(4, 4, 1.0)
                .linearVelocity(1.0, 0.0, 1.0)
                .mass(0.25)
                .sphereCollider(0.08)
                .material(Material.RUBBER), pose -> false, 0.01);

        arena.simulationPeriodic();

        int after = arena.getFieldSimulator().getGamePieces().size();
        assertFalse(after > before);
    }
}
