package jsim.field;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import jsim.api.SimBody;
import jsim.api.SimBodyBuilder;
import jsim.material.Material;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import java.util.Map;

class FieldSimulatorTest {
    private FieldSimulator sim;

    @BeforeEach
    void setUp() {
        sim = new FieldSimulator();
    }

    @Test
    void defaultConstructorCreatesWorkingWorld() {
        assertNotNull(sim.getWorld());
        // 5 field bodies (floor + 4 walls) should already exist
        assertTrue(sim.getWorld().getBodies().size() >= 5);
    }

    @Test
    void gamePieceFallsAndStaysAboveFloor() {
        sim.spawnGamePiece("Note", new SimBodyBuilder("Note0")
            .position(8, 4, 3)
            .mass(0.235)
            .sphereCollider(0.18)
            .material(Material.RUBBER));

        for (int i = 0; i < 200; i++) sim.step();

        GamePiece note = sim.getGamePieces().get(0);
        assertTrue(note.getPose().getZ() >= -0.1,
            "Note Z must stay above floor: " + note.getPose().getZ());
    }

    @Test
    void gamePieceBouncesOffRedWall() {
        GamePiece note = sim.spawnGamePiece("Note", new SimBodyBuilder("Note0")
            .position(8, 4, 0.18)
            .linearVelocity(30, 0, 0)
            .mass(0.235)
            .sphereCollider(0.18)
            .noGravity()
            .material(new Material(0.0, 0.5)));

        for (int i = 0; i < 200; i++) sim.step();

        assertTrue(note.getPose().getX() < FieldLayout.FRC_LENGTH_M + 1.0,
            "Note must not escape past red wall: x=" + note.getPose().getX());
    }

    @Test
    void gamePieceBouncesOffBlueWall() {
        GamePiece note = sim.spawnGamePiece("Note", new SimBodyBuilder("Note0")
            .position(8, 4, 0.18)
            .linearVelocity(-30, 0, 0)
            .mass(0.235)
            .sphereCollider(0.18)
            .noGravity()
            .material(new Material(0.0, 0.5)));

        for (int i = 0; i < 200; i++) sim.step();

        assertTrue(note.getPose().getX() > -1.0,
            "Note must not escape past blue wall: x=" + note.getPose().getX());
    }

    @Test
    void getGamePiecePoses_returnsCorrectCount() {
        sim.spawnGamePiece("Note", new SimBodyBuilder("N0").position(2, 4, 0.18)
            .mass(0.235).sphereCollider(0.18).material(Material.RUBBER));
        sim.spawnGamePiece("Note", new SimBodyBuilder("N1").position(4, 4, 0.18)
            .mass(0.235).sphereCollider(0.18).material(Material.RUBBER));
        sim.spawnGamePiece("Note", new SimBodyBuilder("N2").position(6, 4, 0.18)
            .mass(0.235).sphereCollider(0.18).material(Material.RUBBER));

        assertEquals(3, sim.getGamePiecePoses("Note").length);
    }

    @Test
    void getGamePiecePoses_filtersVariant() {
        sim.spawnGamePiece("Note", new SimBodyBuilder("N0").position(2, 4, 0.18)
            .mass(0.235).sphereCollider(0.18).material(Material.RUBBER));
        sim.spawnGamePiece("Note", new SimBodyBuilder("N1").position(4, 4, 0.18)
            .mass(0.235).sphereCollider(0.18).material(Material.RUBBER));
        sim.spawnGamePiece("Coral", new SimBodyBuilder("C0").position(6, 4, 0.18)
            .mass(0.26).sphereCollider(0.15).material(Material.RUBBER));

        assertEquals(2, sim.getGamePiecePoses("Note").length);
        assertEquals(1, sim.getGamePiecePoses("Coral").length);
        assertEquals(0, sim.getGamePiecePoses("Algae").length);
    }

    @Test
    void getAllGamePiecePoses_groupsByVariant() {
        sim.spawnGamePiece("Note", new SimBodyBuilder("N0").position(2, 4, 0.18)
            .mass(0.235).sphereCollider(0.18).material(Material.RUBBER));
        sim.spawnGamePiece("Coral", new SimBodyBuilder("C0").position(6, 4, 0.18)
            .mass(0.26).sphereCollider(0.15).material(Material.RUBBER));

        Map<String, Pose3d[]> all = sim.getAllGamePiecePoses();
        assertEquals(2, all.size());
        assertEquals(1, all.get("Note").length);
        assertEquals(1, all.get("Coral").length);
    }

    @Test
    
    void removeGamePiece_reducesWorldBodyCount() {
        GamePiece note = sim.spawnGamePiece("Note", new SimBodyBuilder("N0")
            .position(8, 4, 0.18).mass(0.235).sphereCollider(0.18).material(Material.RUBBER));

        int before = sim.getWorld().getBodies().size();
        sim.removeGamePiece(note);

        assertEquals(before - 1, sim.getWorld().getBodies().size());
        assertEquals(0, sim.getGamePieces().size());
    }

    @Test
    void removeGamePiece_releasesFromGripper() {
        SimBody robot = sim.addRobot(new SimBodyBuilder("Robot")
            .position(0, 0, 0.15).mass(50).boxCollider(0.45, 0.45, 0.15)
            .noGravity().fixedRotation().material(Material.CARPET));

        GamePieceGripper gripper = sim.createGripper(robot, new Translation3d(0.5, 0, 0));

        GamePiece note = sim.spawnGamePiece("Note", new SimBodyBuilder("N0")
            .position(0.5, 0, 0.15).mass(0.235).sphereCollider(0.18).material(Material.RUBBER));

        gripper.grab(note);
        assertTrue(gripper.isHolding());

        sim.removeGamePiece(note);
        assertFalse(gripper.isHolding());
    }

    @Test
    void gripperAndStepIntegration() {
        SimBody robot = sim.addRobot(new SimBodyBuilder("Robot")
            .position(0, 0, 0.15).mass(50).boxCollider(0.45, 0.45, 0.15)
            .noGravity().fixedRotation().material(Material.CARPET));

        GamePieceGripper gripper = sim.createGripper(robot, new Translation3d(0.5, 0, 0));

        GamePiece note = sim.spawnGamePiece("Note", new SimBodyBuilder("N0")
            .position(0.5, 0, 0.15).mass(0.235).sphereCollider(0.18).material(Material.RUBBER));

        gripper.grab(note);

        // Move robot; step should teleport the note to follow
        robot.setPose(new Pose3d(new Translation3d(3, 2, 0.15), new Rotation3d()));
        sim.step();

        assertEquals(3.5, note.getPose().getX(), 0.01, "Note should follow robot in X");
        assertEquals(2.0, note.getPose().getY(), 0.01, "Note should follow robot in Y");

        // Eject upward
        gripper.eject(0, 0, 5.0);
        assertFalse(note.isHeld());
        assertEquals(5.0, note.getLinearVelocity().getZ(), 1e-6);
    }

    @Test
    void gamePiecePosesReflectPhysics() {
        sim.spawnGamePiece("Note", new SimBodyBuilder("N0")
            .position(8, 4, 2).mass(0.235).sphereCollider(0.18).material(Material.RUBBER));

        Pose3d[] before = sim.getGamePiecePoses("Note");
        for (int i = 0; i < 10; i++) sim.step();
        Pose3d[] after = sim.getGamePiecePoses("Note");

        // After 10 ticks with gravity, the note should have fallen
        assertTrue(after[0].getZ() < before[0].getZ(),
            "Note should have fallen under gravity");
    }

    @Test
    void customTimestep() {
        FieldSimulator custom = new FieldSimulator(0.005);
        assertEquals(0.005, custom.getWorld().getTimestep(), 1e-12);
    }

    @Test
    void customFieldDimensions() {
        FieldSimulator custom = new FieldSimulator(0.02, 10.0, 5.0);
        // Should create world without throwing and have 5 field bodies
        assertTrue(custom.getWorld().getBodies().size() >= 5);
    }
}
