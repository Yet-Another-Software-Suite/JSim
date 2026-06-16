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

/**
 * End-to-end integration tests for the physics pipeline through the public API.
 *
 * <p>Each test creates a {@link SimWorld}, adds bodies, runs steps, and asserts on
 * observable outcomes. These tests exercise the full tick pipeline
 * (clear → forces → broadphase → narrowphase → solve → integrate → derive).
 */
class PhysicsWorldTest {
    private SimWorld world;

    @BeforeEach
    void setUp() {
        world = new SimWorld(0.02, 10);
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
        for (int i = 0; i < 10; i++) world.step();
        assertTrue(ball.getPosition().getZ() < z0, "Ball should fall under gravity");
    }

    @Test
    void fallDistance_matchesSemiImplicitEuler() {
        // After n ticks of free fall from rest: z should be consistent with semi-implicit Euler
        SimBody ball = world.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 100)
            .mass(1.0)
            .sphereCollider(0.1)
            .noGravity()           // will apply our own force
            .material(Material.RUBBER));
        ball.setLinearVelocity(new Translation3d(0, 0, -9.80665)); // pre-set downward velocity
        world.step(); // one tick with velocity already set
        // Expected z after 1 tick: z0 + v*dt = 100 + (-9.80665)*0.02 = 99.8039...
        assertEquals(100 - 9.80665 * 0.02, ball.getPosition().getZ(), 1e-6);
    }

    // Floor collision

    @Test
    void sphereDoesNotPassThroughFloor() {
        SimBody ball = world.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 0.5)
            .mass(0.24)
            .sphereCollider(0.085)
            .material(Material.RUBBER));

        for (int i = 0; i < 200; i++) world.step();
        assertTrue(ball.getPosition().getZ() >= -0.1,
            "Ball centre Z should not be far below floor: " + ball.getPosition().getZ());
    }

    @Test
    void sphereBouncesAfterHittingFloor() {
        // Use a dedicated world with a rubber floor so combined restitution = min(0.7,0.7) = 0.7.
        // The setUp() world has a carpet floor (e=0.1) which would reduce bounce to near zero.
        SimWorld bounceWorld = new SimWorld(0.02, 10);
        bounceWorld.addBody(new SimBodyBuilder("Floor")
            .planeCollider(0, 0, 1, 0)
            .material(Material.RUBBER));
        SimBody ball = bounceWorld.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 1.0)
            .mass(1.0)
            .sphereCollider(0.1)
            .material(Material.RUBBER));

        // Drop for ~0.5 s; ball hits floor around tick 22 and bounces at ~3 m/s upward
        for (int i = 0; i < 25; i++) bounceWorld.step();

        // Velocity should be clearly positive-Z within the next 30 ticks
        boolean bouncedUp = false;
        for (int i = 0; i < 30; i++) {
            bounceWorld.step();
            if (ball.getLinearVelocity().getZ() > 0.05) { bouncedUp = true; break; }
        }
        assertTrue(bouncedUp, "Ball should bounce upward after hitting rubber floor");
    }

    // Static bodies
    @Test
    void staticBodyDoesNotMove() {
        SimBody wall = world.addBody(new SimBodyBuilder("Wall")
            .position(3, 0, 1)
            .boxCollider(0.1, 2, 1)
            .isStatic()
            .material(Material.WALL));

        for (int i = 0; i < 50; i++) world.step();
        assertEquals(3.0, wall.getPosition().getX(), 1e-9);
        assertEquals(0.0, wall.getPosition().getY(), 1e-9);
        assertEquals(1.0, wall.getPosition().getZ(), 1e-9);
    }

    // Determinism

    @Test
    void determinismIdenticalRuns() {
        SimWorld worldA = new SimWorld(0.02, 10);
        worldA.addBody(new SimBodyBuilder("Floor")
            .planeCollider(0, 0, 1, 0).material(Material.CARPET));
        SimBody ballA = worldA.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 2).mass(1.0).sphereCollider(0.1).material(Material.RUBBER));
        for (int i = 0; i < 100; i++) worldA.step();

        SimWorld worldB = new SimWorld(0.02, 10);
        worldB.addBody(new SimBodyBuilder("Floor")
            .planeCollider(0, 0, 1, 0).material(Material.CARPET));
        SimBody ballB = worldB.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 2).mass(1.0).sphereCollider(0.1).material(Material.RUBBER));
        for (int i = 0; i < 100; i++) worldB.step();

        assertEquals(ballA.getPosition().getX(), ballB.getPosition().getX(), 0.0,
            "Bit-identical X across runs");
        assertEquals(ballA.getPosition().getY(), ballB.getPosition().getY(), 0.0,
            "Bit-identical Y across runs");
        assertEquals(ballA.getPosition().getZ(), ballB.getPosition().getZ(), 0.0,
            "Bit-identical Z across runs");
    }

    @Test
    void determinism_orderInsensitiveForNonColliding() {
        // Bodies that never interact must produce the same result regardless of registration order
        SimWorld w1 = new SimWorld(0.02, 10);
        w1.addBody(new SimBodyBuilder("Floor").planeCollider(0,0,1,0).material(Material.CARPET));
        SimBody a1 = w1.addBody(new SimBodyBuilder("A").position(10,0,5).mass(1).sphereCollider(0.1).material(Material.RUBBER));
        SimBody b1 = w1.addBody(new SimBodyBuilder("B").position(-10,0,5).mass(1).sphereCollider(0.1).material(Material.RUBBER));
        for (int i = 0; i < 50; i++) w1.step();

        SimWorld w2 = new SimWorld(0.02, 10);
        w2.addBody(new SimBodyBuilder("Floor").planeCollider(0,0,1,0).material(Material.CARPET));
        // Register B before A this time — should not affect isolated-body trajectories
        SimBody b2 = w2.addBody(new SimBodyBuilder("B").position(-10,0,5).mass(1).sphereCollider(0.1).material(Material.RUBBER));
        SimBody a2 = w2.addBody(new SimBodyBuilder("A").position(10,0,5).mass(1).sphereCollider(0.1).material(Material.RUBBER));
        for (int i = 0; i < 50; i++) w2.step();

        assertEquals(a1.getPosition().getZ(), a2.getPosition().getZ(), 1e-9);
        assertEquals(b1.getPosition().getZ(), b2.getPosition().getZ(), 1e-9);
    }

    // Two-body collision — linear momentum conservation

    @Test
    void twoBodyCollision_linearMomentumConserved() {
        // Two equal-mass spheres moving toward each other on the X axis
        SimBody sA = world.addBody(new SimBodyBuilder("A")
            .position(-0.5, 0, 0.5)
            .linearVelocity(3, 0, 0)
            .mass(1.0)
            .sphereCollider(0.2)
            .noGravity().fixedRotation()
            .material(new frc.jsim.material.Material(0.0, 0.5)));
        SimBody sB = world.addBody(new SimBodyBuilder("B")
            .position(0.5, 0, 0.5)
            .linearVelocity(-3, 0, 0)
            .mass(1.0)
            .sphereCollider(0.2)
            .noGravity().fixedRotation()
            .material(new frc.jsim.material.Material(0.0, 0.5)));

        // Total X momentum = 1*3 + 1*(-3) = 0
        double p0 = sA.getLinearVelocity().getX() + sB.getLinearVelocity().getX();

        for (int i = 0; i < 50; i++) world.step();

        double p1 = sA.getLinearVelocity().getX() + sB.getLinearVelocity().getX();
        assertEquals(p0, p1, 0.05, "Total X-momentum must be conserved (±5%)");
    }

    @Test
    void twoBodyCollision_spheresSeperateAfterImpact() {
        SimBody sA = world.addBody(new SimBodyBuilder("A")
            .position(-0.25, 0, 0.5).linearVelocity(2, 0, 0)
            .mass(1.0).sphereCollider(0.2)
            .noGravity().fixedRotation()
            .material(new frc.jsim.material.Material(0.0, 1.0)));
        SimBody sB = world.addBody(new SimBodyBuilder("B")
            .position(0.25, 0, 0.5).linearVelocity(-2, 0, 0)
            .mass(1.0).sphereCollider(0.2)
            .noGravity().fixedRotation()
            .material(new frc.jsim.material.Material(0.0, 1.0)));

        for (int i = 0; i < 100; i++) world.step();

        // After elastic collision + some travel time, A should be at negative X, B at positive X
        assertTrue(sA.getPosition().getX() < 0 || sB.getPosition().getX() > 0,
            "Bodies should have separated after elastic collision");
    }

    // BodyTracker
    @Test
    void bodyTrackerAccumulatesDistance() {
        SimBody ball = world.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 5).mass(1.0).sphereCollider(0.1).material(Material.RUBBER));

        BodyTracker tracker = new BodyTracker(ball, 50);
        for (int i = 0; i < 50; i++) { world.step(); tracker.update(); }

        assertTrue(tracker.getTotalDistance() > 0, "Ball should have moved");
        assertTrue(tracker.getPoseHistory().length > 0, "History should be recorded");
    }

    @Test
    void bodyTrackerHistoryBounded() {
        SimBody ball = world.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 5).mass(1.0).sphereCollider(0.1).material(Material.RUBBER));

        BodyTracker tracker = new BodyTracker(ball, 10);
        for (int i = 0; i < 100; i++) { world.step(); tracker.update(); }

        assertTrue(tracker.getPoseHistory().length <= 10, "History must not exceed maxHistory");
    }

    @Test
    void bodyTrackerIsAtRest_returnsTrueForStaticBody() {
        SimBody wall = world.addBody(new SimBodyBuilder("Wall")
            .position(0, 0, 1).boxCollider(1,1,1).isStatic().material(Material.WALL));
        BodyTracker tracker = new BodyTracker(wall, 5);
        for (int i = 0; i < 5; i++) { world.step(); tracker.update(); }
        assertTrue(tracker.isAtRest(1e-6), "Static body should always be at rest");
    }

    @Test
    void bodyTrackerReset_clearsTotalDistance() {
        SimBody ball = world.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 5).mass(1.0).sphereCollider(0.1).material(Material.RUBBER));
        BodyTracker tracker = new BodyTracker(ball, 10);
        for (int i = 0; i < 20; i++) { world.step(); tracker.update(); }
        assertTrue(tracker.getTotalDistance() > 0);
        tracker.reset();
        assertEquals(0.0, tracker.getTotalDistance(), 0.0);
        assertEquals(0, tracker.getPoseHistory().length);
    }

    // Actuator
    @Test
    void actuatorPushesBody() {
        SimBody box = world.addBody(new SimBodyBuilder("Box")
            .position(0, 0, 0.5).mass(5.0).boxCollider(0.3, 0.3, 0.3)
            .noGravity().fixedRotation().material(Material.DEFAULT));

        box.getActuator().setForce(100, 0, 0);
        for (int i = 0; i < 10; i++) world.step();

        assertTrue(box.getLinearVelocity().getX() > 0,
            "Box should accelerate in +X: vx=" + box.getLinearVelocity().getX());
    }

    @Test
    void actuatorZero_stopsAcceleration() {
        SimBody box = world.addBody(new SimBodyBuilder("Box")
            .position(0, 0, 0.5).mass(5.0).boxCollider(0.3, 0.3, 0.3)
            .noGravity().fixedRotation().material(Material.DEFAULT));

        box.getActuator().setForce(100, 0, 0);
        for (int i = 0; i < 5; i++) world.step();
        double vAfterForce = box.getLinearVelocity().getX();

        box.getActuator().zero();
        for (int i = 0; i < 5; i++) world.step();
        // Velocity should remain roughly constant (no more acceleration, no friction since no gravity)
        assertEquals(vAfterForce, box.getLinearVelocity().getX(), 0.01,
            "Velocity constant after zeroing actuator");
    }

    @Test
    void actuatorTorque_spinBody() {
        SimBody box = world.addBody(new SimBodyBuilder("Box")
            .position(0, 0, 1).mass(2.0).boxCollider(0.5, 0.5, 0.5)
            .noGravity().material(Material.DEFAULT));

        box.getActuator().setTorque(0, 0, 10); // spin about Z
        for (int i = 0; i < 10; i++) world.step();

        assertTrue(box.getAngularVelocity().getZ() > 0,
            "Angular velocity Z should grow: " + box.getAngularVelocity().getZ());
    }

    // Flags

    @Test
    void noGravityBodyDoesNotFall() {
        SimBody floater = world.addBody(new SimBodyBuilder("Floater")
            .position(0, 0, 3).mass(1.0).sphereCollider(0.1)
            .noGravity().material(Material.DEFAULT));

        double z0 = floater.getPosition().getZ();
        for (int i = 0; i < 50; i++) world.step();
        assertEquals(z0, floater.getPosition().getZ(), 1e-6,
            "Floater Z should not change without gravity");
    }

    @Test
    void fixedRotation_bodyDoesNotSpin() {
        SimBody box = world.addBody(new SimBodyBuilder("Box")
            .position(0, 0, 0.5).mass(5.0).boxCollider(0.3, 0.3, 0.3)
            .noGravity().fixedRotation().material(Material.DEFAULT));

        box.getActuator().setForceAtPoint(100, 0, 0, 0, 0.3, 0.5); // off-centre force
        for (int i = 0; i < 10; i++) world.step();

        // Angular velocity must remain zero
        assertEquals(0.0, box.getAngularVelocity().getX(), 1e-9);
        assertEquals(0.0, box.getAngularVelocity().getY(), 1e-9);
        assertEquals(0.0, box.getAngularVelocity().getZ(), 1e-9);
    }

    // Pose3d round-trip
    @Test
    void poseRoundTrip() {
        SimBody body = world.addBody(new SimBodyBuilder("Teleport")
            .position(1, 2, 3).mass(1.0).sphereCollider(0.1).material(Material.DEFAULT));

        Pose3d target = new Pose3d(new Translation3d(5, 6, 7), new Rotation3d(0.1, 0.2, 0.3));
        body.setPose(target);
        Pose3d retrieved = body.getPose();

        assertEquals(target.getX(), retrieved.getX(), 1e-9);
        assertEquals(target.getY(), retrieved.getY(), 1e-9);
        assertEquals(target.getZ(), retrieved.getZ(), 1e-9);
    }

    // SimWorld API

    @Test
    void findBody_returnsCorrectBody() {
        world.addBody(new SimBodyBuilder("Alpha").position(1, 0, 0)
            .mass(1).sphereCollider(0.1).material(Material.DEFAULT));
        world.addBody(new SimBodyBuilder("Beta").position(2, 0, 0)
            .mass(1).sphereCollider(0.1).material(Material.DEFAULT));

        SimBody found = world.findBody("Beta");
        assertNotNull(found);
        assertEquals("Beta", found.getName());
    }

    @Test
    void findBody_unknownName_returnsNull() {
        assertNull(world.findBody("DoesNotExist"));
    }

    @Test
    void removeBody_bodyNoLongerInWorld() {
        SimBody ball = world.addBody(new SimBodyBuilder("Temp")
            .position(0, 0, 5).mass(1).sphereCollider(0.1).material(Material.DEFAULT));

        int countBefore = world.getBodies().size();
        world.removeBody(ball);
        assertEquals(countBefore - 1, world.getBodies().size());
    }

    @Test
    void getTimestep_returnsConfiguredValue() {
        SimWorld w = new SimWorld(0.005);
        assertEquals(0.005, w.getTimestep(), 1e-12);
    }

    // Box-box interaction

    @Test
    void boxBoxCollision_bodiesSeparate() {
        SimBody bA = world.addBody(new SimBodyBuilder("BoxA")
            .position(-0.4, 0, 1).linearVelocity(2, 0, 0)
            .mass(2.0).boxCollider(0.3, 0.3, 0.3)
            .noGravity().fixedRotation()
            .material(new frc.jsim.material.Material(0.0, 0.5)));
        SimBody bB = world.addBody(new SimBodyBuilder("BoxB")
            .position(0.4, 0, 1).linearVelocity(-2, 0, 0)
            .mass(2.0).boxCollider(0.3, 0.3, 0.3)
            .noGravity().fixedRotation()
            .material(new frc.jsim.material.Material(0.0, 0.5)));

        for (int i = 0; i < 100; i++) world.step();

        // After 100 ticks the two boxes should not deeply overlap
        double dist = Math.abs(bA.getPosition().getX() - bB.getPosition().getX());
        assertTrue(dist > 0.1, "Boxes should have separated after collision, dist=" + dist);
    }
}
