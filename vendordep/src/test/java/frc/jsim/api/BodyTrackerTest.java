package frc.jsim.api;

import static org.junit.jupiter.api.Assertions.*;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.jsim.material.Material;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Isolated unit tests for {@link BodyTracker}.
 * Complement the integration-level tests in {@code PhysicsWorldTest}.
 */
class BodyTrackerTest {

    private SimWorld world;
    private SimBody ball;

    @BeforeEach
    void setUp() {
        world = new SimWorld(0.02, 10);
        world.addBody(new SimBodyBuilder("Floor").planeCollider(0, 0, 1, 0).material(Material.CARPET));
        ball = world.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 5).mass(Kilograms.of(1)).sphereCollider(0.1).material(Material.RUBBER));
    }

    @Test
    void initialTotalDistance_isZero() {
        BodyTracker tracker = new BodyTracker(ball, 10);
        assertEquals(0.0, tracker.getTotalDistance().in(Meters), 1e-12);
    }

    @Test
    void initialPoseHistory_isEmpty() {
        BodyTracker tracker = new BodyTracker(ball, 10);
        assertEquals(0, tracker.getPoseHistory().length);
    }

    @Test
    void getTarget_returnsTrackedBody() {
        BodyTracker tracker = new BodyTracker(ball, 5);
        assertSame(ball, tracker.getTarget());
    }

    @Test
    void getPose_matchesBallPoseBeforeUpdate() {
        BodyTracker tracker = new BodyTracker(ball, 5);
        assertEquals(ball.getPose().getX(), tracker.getPose().getX(), 1e-9);
        assertEquals(ball.getPose().getZ(), tracker.getPose().getZ(), 1e-9);
    }

    @Test
    void getVelocity_matchesBallVelocityBeforeUpdate() {
        BodyTracker tracker = new BodyTracker(ball, 5);
        assertEquals(ball.getLinearVelocity().getZ(), tracker.getVelocity().getZ(), 1e-9);
    }

    @Test
    void zeroMaxHistory_noHistoryStored_butDistanceAccumulates() {
        BodyTracker tracker = new BodyTracker(ball); // maxHistory=0
        for (int i = 0; i < 20; i++) {
            world.step();
            tracker.update();
        }
        assertEquals(0, tracker.getPoseHistory().length, "No history should be stored with maxHistory=0");
        assertEquals(0, tracker.getVelocityHistory().length);
        assertTrue(tracker.getTotalDistance().in(Meters) > 0, "Distance should still accumulate");
    }

    @Test
    void isAtRest_trueForZeroVelocityBody() {
        // Ball starts at rest (no initial velocity)
        ball.setLinearVelocity(0, 0, 0);
        BodyTracker tracker = new BodyTracker(ball, 5);
        tracker.update(); // record one sample
        assertTrue(tracker.isAtRest(MetersPerSecond.of(1.0)));
    }

    @Test
    void isAtRest_falseForMovingBody() {
        ball.setLinearVelocity(5, 0, 0);
        BodyTracker tracker = new BodyTracker(ball, 5);
        world.step();
        tracker.update();
        assertFalse(tracker.isAtRest(MetersPerSecond.of(0.01)));
    }

    @Test
    void velocityHistory_boundedByMaxHistory() {
        BodyTracker tracker = new BodyTracker(ball, 3);
        for (int i = 0; i < 10; i++) {
            world.step();
            tracker.update();
        }
        assertTrue(tracker.getVelocityHistory().length <= 3);
    }

    @Test
    void reset_clearsVelocityHistory() {
        BodyTracker tracker = new BodyTracker(ball, 5);
        for (int i = 0; i < 5; i++) {
            world.step();
            tracker.update();
        }
        tracker.reset();
        assertEquals(0, tracker.getVelocityHistory().length);
    }

    @Test
    void poseHistorySnapshot_isIndependentOfInternalState() {
        BodyTracker tracker = new BodyTracker(ball, 5);
        world.step();
        tracker.update();
        var snapshot1 = tracker.getPoseHistory();
        world.step();
        tracker.update();
        var snapshot2 = tracker.getPoseHistory();
        // Modifying snapshot1 should not affect snapshot2
        assertNotSame(snapshot1, snapshot2);
    }

    @Test
    void update_reflectsLatestPose() {
        BodyTracker tracker = new BodyTracker(ball, 5);
        double zBefore = ball.getPosition().getZ();
        for (int i = 0; i < 10; i++) {
            world.step();
            tracker.update();
        }
        // Ball has fallen — tracker should reflect new Z
        assertTrue(tracker.getPose().getZ() < zBefore,
            "Tracker pose should follow body as it falls");
    }
}
