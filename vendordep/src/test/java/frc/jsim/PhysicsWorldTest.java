package frc.jsim;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.jsim.api.BodyTracker;
import frc.jsim.api.SimBody;
import frc.jsim.api.SimBodyBuilder;
import frc.jsim.api.SimWorld;
import frc.jsim.material.Material;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PhysicsWorldTest {
    private SimWorld world;

    @BeforeEach
    void setUp() {
        world = new SimWorld(0.02, 10);
        // Infinite carpet floor at Z = 0, normal pointing up
        world.addBody(new SimBodyBuilder("Floor")
            .planeCollider(0, 0, 1, 0)
            .material(Material.CARPET));
    }

    // Gravity
    @Test
    void bodyFallsUnderGravity() {
        SimBody ball = world.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 5)
            .mass(1.0)
            .sphereCollider(0.1)
            .material(Material.RUBBER));

        double z0 = ball.getPosition().getZ();
        // Step 10 ticks (~0.2 s)
        for (int i = 0; i < 10; i++) world.step();
        double z1 = ball.getPosition().getZ();
        assertTrue(z1 < z0, "Ball should fall under gravity");
    }

    // Collision with floor — ball should not pass through
    @Test
    void sphereDoesNotPassThroughFloor() {
        SimBody ball = world.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 0.5)
            .mass(0.24)
            .sphereCollider(0.085)
            .material(Material.RUBBER));

        // Run long enough that the ball would have sunk far below floor without collision
        for (int i = 0; i < 200; i++) world.step();

        double z = ball.getPosition().getZ();
        // Centre must stay above floor - radius = 0.085 m
        assertTrue(z >= -0.1, "Ball centre Z should not be far below floor: " + z);
    }

    // Static body never moves
    @Test
    void staticBodyDoesNotMove() {
        SimBody wall = world.addBody(new SimBodyBuilder("Wall")
            .position(3, 0, 1)
            .boxCollider(0.1, 2, 1)
            .isStatic()
            .material(Material.WALL));

        for (int i = 0; i < 50; i++) world.step();

        Translation3d pos = wall.getPosition();
        assertEquals(3.0, pos.getX(), 1e-9);
        assertEquals(0.0, pos.getY(), 1e-9);
        assertEquals(1.0, pos.getZ(), 1e-9);
    }

    // Determinism — two identical runs produce identical results
    @Test
    void determinismIdenticalRuns() {
        SimBody ball = world.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 2)
            .mass(1.0)
            .sphereCollider(0.1)
            .material(Material.RUBBER));

        // Run A
        SimWorld worldA = new SimWorld(0.02, 10);
        worldA.addBody(new SimBodyBuilder("Floor")
            .planeCollider(0, 0, 1, 0).material(Material.CARPET));
        SimBody ballA = worldA.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 2).mass(1.0).sphereCollider(0.1).material(Material.RUBBER));
        for (int i = 0; i < 100; i++) worldA.step();

        // Run B (fresh world, identical setup)
        SimWorld worldB = new SimWorld(0.02, 10);
        worldB.addBody(new SimBodyBuilder("Floor")
            .planeCollider(0, 0, 1, 0).material(Material.CARPET));
        SimBody ballB = worldB.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 2).mass(1.0).sphereCollider(0.1).material(Material.RUBBER));
        for (int i = 0; i < 100; i++) worldB.step();

        assertEquals(ballA.getPosition().getX(), ballB.getPosition().getX(), 0.0);
        assertEquals(ballA.getPosition().getY(), ballB.getPosition().getY(), 0.0);
        assertEquals(ballA.getPosition().getZ(), ballB.getPosition().getZ(), 0.0);
    }

    // ------------------------------------------------------------------
    // BodyTracker
    // ------------------------------------------------------------------

    @Test
    void bodyTrackerAccumulatesDistance() {
        SimBody ball = world.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 5)
            .mass(1.0)
            .sphereCollider(0.1)
            .material(Material.RUBBER));

        BodyTracker tracker = new BodyTracker(ball, 50);
        for (int i = 0; i < 50; i++) {
            world.step();
            tracker.update();
        }

        assertTrue(tracker.getTotalDistance() > 0, "Ball should have moved");
        assertTrue(tracker.getPoseHistory().length > 0, "History should be recorded");
    }

    // ------------------------------------------------------------------
    // Actuator applies force
    // ------------------------------------------------------------------

    @Test
    void actuatorPushesBody() {
        SimBody box = world.addBody(new SimBodyBuilder("Box")
            .position(0, 0, 0.5)
            .mass(5.0)
            .boxCollider(0.3, 0.3, 0.3)
            .noGravity()
            .fixedRotation()
            .material(Material.DEFAULT));

        // Push 100 N in +X
        box.getActuator().setForce(100, 0, 0);
        for (int i = 0; i < 10; i++) world.step();

        double vx = box.getLinearVelocity().getX();
        assertTrue(vx > 0, "Box should accelerate in +X: vx=" + vx);
    }

    // ------------------------------------------------------------------
    // No-gravity flag
    // ------------------------------------------------------------------

    @Test
    void noGravityBodyDoesNotFall() {
        SimBody floater = world.addBody(new SimBodyBuilder("Floater")
            .position(0, 0, 3)
            .mass(1.0)
            .sphereCollider(0.1)
            .noGravity()
            .material(Material.DEFAULT));

        double z0 = floater.getPosition().getZ();
        for (int i = 0; i < 50; i++) world.step();
        double z1 = floater.getPosition().getZ();
        assertEquals(z0, z1, 1e-6, "Floater Z should not change without gravity");
    }

    // ------------------------------------------------------------------
    // Pose3d round-trip through setPose / getPose
    // ------------------------------------------------------------------

    @Test
    void poseRoundTrip() {
        SimBody body = world.addBody(new SimBodyBuilder("Teleport")
            .position(1, 2, 3)
            .mass(1.0)
            .sphereCollider(0.1)
            .material(Material.DEFAULT));

        Pose3d target = new Pose3d(new Translation3d(5, 6, 7), new Rotation3d(0.1, 0.2, 0.3));
        body.setPose(target);
        Pose3d retrieved = body.getPose();

        assertEquals(target.getX(), retrieved.getX(), 1e-9);
        assertEquals(target.getY(), retrieved.getY(), 1e-9);
        assertEquals(target.getZ(), retrieved.getZ(), 1e-9);
    }
}
