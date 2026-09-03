package jsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import jsim.physics.layers.gamepieces.Fuel2026;
import jsim.physics.layers.gamepieces.Gamepiece;
import org.junit.jupiter.api.Test;

class GamepieceTest {

  @Test
  void fuelInheritsCommonGamepieceStateAndCanAttachToRobot() {
    Fuel2026 fuel = new Fuel2026(new Translation3d());
    Gamepiece piece = fuel;

    piece.attachToRobot(
        new Pose2d(1.0, 2.0, Rotation2d.kCCW_90deg),
        new Transform3d(1.0, 0.0, 0.5, Rotation3d.kZero));

    assertTrue(piece.isAttachedToRobot());
    assertEquals(1.0, piece.getPosition().getX(), 1e-9);
    assertEquals(3.0, piece.getPosition().getY(), 1e-9);
    assertEquals(0.5, piece.getPosition().getZ(), 1e-9);

    piece.updateRobotAttachment(new Pose2d(2.0, 2.0, Rotation2d.kZero));
    assertEquals(3.0, piece.getPosition().getX(), 1e-9);
    assertEquals(2.0, piece.getPosition().getY(), 1e-9);

    piece.detachFromRobot();
    assertTrue(!piece.isAttachedToRobot());
  }
}