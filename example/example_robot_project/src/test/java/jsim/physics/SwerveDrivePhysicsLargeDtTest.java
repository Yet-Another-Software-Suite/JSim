package jsim.physics;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import jsim.physics.layers.Dyn4jCollisionLayer;
import jsim.physics.layers.FieldLayout;
import jsim.physics.layers.fields.Field2026;

import org.junit.jupiter.api.Test;

/**
 * Regression tests proving {@link SwerveDrivePhysics#update} doesn't tunnel a robot through thin
 * field elements (TOWER UPRIGHTS, TRENCH GATEs) when a single call covers an unusually large
 * {@code dtSeconds} -- e.g. a debugger breakpoint, a sim GUI pause, or just a slow frame, all of
 * which show up as a real elapsed-time spike when {@link SwerveDrivePhysics#update()} (no-arg)
 * derives {@code dtSeconds} from {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.
 *
 * <p>dyn4j's collision detection is discrete: it only checks for overlap at the end of a step, not
 * along the swept path. A single big step (e.g. 4 m/s for 0.5s = 2m of travel) can skip clean over
 * an element far thinner than that in one shot. {@code update(...)} internally chunks a large
 * {@code dtSeconds} into several {@code DEFAULT_DT_SECONDS}-sized substeps to prevent this.
 */
class SwerveDrivePhysicsLargeDtTest {

  private static final Field2026 FIELD = new Field2026();
  private static final double ROBOT_SIZE = 0.9;
  private static final double ROBOT_HALF = ROBOT_SIZE / 2.0;
  private static final double APPROACH_SPEED = 4.0; // m/s

  private static double[] bounds(FieldLayout.Element element) {
    double minX = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;
    for (Translation2d v : element.getVertices()) {
      minX = Math.min(minX, v.getX());
      maxX = Math.max(maxX, v.getX());
      minY = Math.min(minY, v.getY());
      maxY = Math.max(maxY, v.getY());
    }
    return new double[] {minX, maxX, minY, maxY};
  }

  private static SwerveDrivePhysics newPhysics(Pose2d initialPose) {
    Translation2d[] moduleLocations = {
        new Translation2d(0.3, 0.3), new Translation2d(0.3, -0.3),
        new Translation2d(-0.3, 0.3), new Translation2d(-0.3, -0.3)
    };
    SwerveDrivePhysics physics = new SwerveDrivePhysics(
        moduleLocations, Meters.of(ROBOT_SIZE), Meters.of(ROBOT_SIZE), initialPose, zeroModulePositions());
    return physics.addLayer(new Dyn4jCollisionLayer(Kilograms.of(50.0), FIELD));
  }

  private static SwerveModulePosition[] zeroModulePositions() {
    return new SwerveModulePosition[] {
        new SwerveModulePosition(0, new Rotation2d()), new SwerveModulePosition(0, new Rotation2d()),
        new SwerveModulePosition(0, new Rotation2d()), new SwerveModulePosition(0, new Rotation2d())
    };
  }

  @Test
  void singleHugeDtStepDoesNotTunnelThroughTowerUpright() {
    // 4 m/s for 0.5s in one un-chunked step would be 2m of travel -- more than enough to sail
    // clean through the upright's ~0.089m depth (and the whole field) without substepping.
    double[] uprightBounds = bounds(FIELD.getBlueTowerUprights()[0]);
    double uprightCenterY = (uprightBounds[2] + uprightBounds[3]) / 2.0;
    Pose2d start = new Pose2d(uprightBounds[1] + 1.0, uprightCenterY, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    SwerveDrivePhysics.PhysicsState state = physics.update(
        new ChassisSpeeds(-APPROACH_SPEED, 0, 0), Rotation2d.kZero, zeroModulePositions(), 0.5);

    double expectedStopX = uprightBounds[1] + ROBOT_HALF;
    assertTrue(state.pose().getX() >= expectedStopX - 0.25,
        "Robot tunneled through the TOWER upright on a single large-dt step: " + state.pose());
  }

  @Test
  void singleHugeDtStepDoesNotTunnelThroughTrenchGate() {
    double[] gateBounds = bounds(FIELD.getBlueSouthTrenchGate());
    double gateCenterX = (gateBounds[0] + gateBounds[1]) / 2.0;
    Pose2d start = new Pose2d(gateCenterX, gateBounds[3] + 1.0, Rotation2d.kZero);
    SwerveDrivePhysics physics = newPhysics(start);

    SwerveDrivePhysics.PhysicsState state = physics.update(
        new ChassisSpeeds(0, -APPROACH_SPEED, 0), Rotation2d.kZero, zeroModulePositions(), 0.5);

    double expectedStopY = gateBounds[3] + ROBOT_HALF;
    assertTrue(state.pose().getY() >= expectedStopY - 0.25,
        "Robot tunneled through the TRENCH gate on a single large-dt step: " + state.pose());
  }
}
