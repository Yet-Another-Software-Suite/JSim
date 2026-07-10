package jsim.field;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import jsim.api.SimBody;
import jsim.api.SimBodyBuilder;
import jsim.api.SimWorld;
import jsim.material.Material;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class GamePieceGripperTest {
    private SimWorld world;
    private SimBody robot;
    private GamePiece piece;
    private GamePieceGripper gripper;

    @BeforeEach
    void setUp() {
        world = new SimWorld(0.02, 10);

        robot = world.addBody(new SimBodyBuilder("Robot")
            .position(0, 0, 0.15)
            .mass(50)
            .boxCollider(0.45, 0.45, 0.15)
            .noGravity().fixedRotation()
            .material(Material.CARPET));

        SimBody pieceBody = world.addBody(new SimBodyBuilder("Note")
            .position(5, 0, 0.18)
            .mass(0.235)
            .sphereCollider(0.18)
            .material(Material.RUBBER));

        piece   = new GamePiece(pieceBody, "Note");
        gripper = new GamePieceGripper(robot, new Translation3d(0.5, 0, 0));
    }

    @Test
    void initiallyNotHolding() {
        assertFalse(gripper.isHolding());
        assertNull(gripper.getHeld());
        assertFalse(piece.isHeld());
    }

    @Test
    void grabAttachesPiece() {
        assertTrue(gripper.grab(piece));
        assertTrue(gripper.isHolding());
        assertSame(piece, gripper.getHeld());
        assertTrue(piece.isHeld());
    }

    @Test
    void grabSnapsToIntakePosition() {
        gripper.grab(piece);
        // Intake offset is (0.5, 0, 0) in robot-local frame; robot is at (0,0,0.15) facing +X
        Pose3d pose = piece.getPose();
        assertEquals(0.5, pose.getX(), 1e-6, "piece X should equal robot X + intake offset");
        assertEquals(0.0, pose.getY(), 1e-6);
        assertEquals(0.15, pose.getZ(), 1e-6);
    }

    @Test
    void grabMatchesRobotVelocity() {
        robot.setLinearVelocity(2.0, 1.0, 0);
        gripper.grab(piece);
        Translation3d pv = piece.getLinearVelocity();
        assertEquals(2.0, pv.getX(), 1e-6);
        assertEquals(1.0, pv.getY(), 1e-6);
        assertEquals(0.0, pv.getZ(), 1e-6);
    }

    @Test
    void grabReturnsFalseIfAlreadyHolding() {
        SimBody otherBody = world.addBody(new SimBodyBuilder("Other")
            .position(3, 0, 0.18).mass(0.235).sphereCollider(0.18).material(Material.RUBBER));
        GamePiece other = new GamePiece(otherBody, "Note");

        gripper.grab(piece);
        assertFalse(gripper.grab(other), "Second grab should fail while already holding");
        assertFalse(other.isHeld());
    }

    @Test
    void grabReturnsFalseIfPieceAlreadyHeld() {
        GamePieceGripper gripper2 = new GamePieceGripper(robot, new Translation3d(-0.5, 0, 0));
        gripper.grab(piece);
        assertFalse(gripper2.grab(piece), "Cannot grab a piece already held elsewhere");
    }

    @Test
    void releaseDetachesPiece() {
        gripper.grab(piece);
        gripper.release();
        assertFalse(gripper.isHolding());
        assertNull(gripper.getHeld());
        assertFalse(piece.isHeld());
    }

    @Test
    void releaseGivesRobotVelocity() {
        robot.setLinearVelocity(3.0, 0.5, 0);
        gripper.grab(piece);
        gripper.release();
        Translation3d pv = piece.getLinearVelocity();
        assertEquals(3.0, pv.getX(), 1e-6);
        assertEquals(0.5, pv.getY(), 1e-6);
    }

    @Test
    void releaseNoOpIfNotHolding() {
        assertDoesNotThrow(() -> gripper.release());
        assertFalse(gripper.isHolding());
    }

    @Test
    void ejectSetsCustomVelocity() {
        gripper.grab(piece);
        gripper.eject(0, 0, 8.0);
        assertFalse(piece.isHeld());
        assertNull(gripper.getHeld());
        Translation3d pv = piece.getLinearVelocity();
        assertEquals(0.0, pv.getX(), 1e-6);
        assertEquals(0.0, pv.getY(), 1e-6);
        assertEquals(8.0, pv.getZ(), 1e-6);
    }

    @Test
    void ejectNoOpIfNotHolding() {
        assertDoesNotThrow(() -> gripper.eject(0, 0, 8));
    }

    @Test
    void updateTeleportsPieceWhenRobotMoves() {
        gripper.grab(piece);

        // Move robot to a new position
        robot.setPose(new Pose3d(new Translation3d(4, 2, 0.15), new Rotation3d()));
        gripper.update();

        // Piece should be at robot + (0.5, 0, 0)
        assertEquals(4.5, piece.getPose().getX(), 1e-6, "Piece X should track robot");
        assertEquals(2.0, piece.getPose().getY(), 1e-6);
        assertEquals(0.15, piece.getPose().getZ(), 1e-6);
    }

    @Test
    void updateNoOpIfNotHolding() {
        assertDoesNotThrow(() -> gripper.update());
    }

    @Test
    void intakeOffsetRotatesWithRobot() {
        // Rotate robot 90° about Z so its +X axis becomes world +Y
        robot.setPose(new Pose3d(
            new Translation3d(0, 0, 0.15),
            new Rotation3d(0, 0, Math.PI / 2)));

        gripper.grab(piece);

        // Intake offset (0.5, 0, 0) in robot frame → (0, 0.5, 0) in world frame
        assertEquals(0.0, piece.getPose().getX(), 1e-6, "X should stay at 0");
        assertEquals(0.5, piece.getPose().getY(), 1e-6, "Y should be 0.5 (rotated offset)");
    }
}
