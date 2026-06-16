package frc.jsim.api;

import java.util.ArrayDeque;
import java.util.Deque;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Lightweight utility for querying and optionally recording the history of a
 * {@link SimBody}'s pose and velocity over time.
 *
 * <p>Call {@link #update()} each tick (after {@code world.step()}) to record the
 * current sample. History depth is bounded by {@code maxHistory} to avoid unbounded
 * memory use during long simulations.
 *
 * <p>Typical usage — follow a game piece:
 * <pre>{@code
 * BodyTracker ballTracker = new BodyTracker(ball, 100);
 *
 * // In simulationPeriodic():
 * world.step();
 * ballTracker.update();
 * Pose3d current = ballTracker.getPose();
 * }</pre>
 */
public final class BodyTracker {
    private final SimBody target;
    private final int maxHistory;
    private final Deque<Pose3d> poseHistory;
    private final Deque<Translation3d> velocityHistory;

    private Pose3d latestPose;
    private Translation3d latestVelocity;
    private double totalDistance;

    public BodyTracker(SimBody target) {
        this(target, 0);
    }

    /**
     * @param target     the body to track
     * @param maxHistory maximum number of past samples to retain (0 = no history)
     */
    public BodyTracker(SimBody target, int maxHistory) {
        this.target = target;
        this.maxHistory = maxHistory;
        this.poseHistory = new ArrayDeque<>(maxHistory + 1);
        this.velocityHistory = new ArrayDeque<>(maxHistory + 1);
        this.latestPose = target.getPose();
        this.latestVelocity = target.getLinearVelocity();
        this.totalDistance = 0;
    }

    // ------------------------------------------------------------------
    // Update
    // ------------------------------------------------------------------

    /**
     * Record the body's current state. Call once per tick after the physics step.
     */
    public void update() {
        Pose3d prevPose = latestPose;
        latestPose = target.getPose();
        latestVelocity = target.getLinearVelocity();

        // Accumulate distance travelled
        Translation3d prev = prevPose.getTranslation();
        Translation3d curr = latestPose.getTranslation();
        double dx = curr.getX() - prev.getX();
        double dy = curr.getY() - prev.getY();
        double dz = curr.getZ() - prev.getZ();
        totalDistance += Math.sqrt(dx * dx + dy * dy + dz * dz);

        if (maxHistory > 0) {
            poseHistory.addLast(latestPose);
            velocityHistory.addLast(latestVelocity);
            while (poseHistory.size() > maxHistory) poseHistory.pollFirst();
            while (velocityHistory.size() > maxHistory) velocityHistory.pollFirst();
        }
    }

    // ------------------------------------------------------------------
    // Current state
    // ------------------------------------------------------------------

    /** The body being tracked. */
    public SimBody getTarget() { return target; }

    /** Latest recorded pose (updated by {@link #update()}). */
    public Pose3d getPose() { return latestPose; }

    /** Latest recorded linear velocity. */
    public Translation3d getVelocity() { return latestVelocity; }

    /** Latest recorded speed (m/s). */
    public double getSpeed() { return target.getSpeed(); }

    /** Total distance travelled since construction (metres). */
    public double getTotalDistance() { return totalDistance; }

    // ------------------------------------------------------------------
    // History
    // ------------------------------------------------------------------

    /**
     * Ordered list of past poses, oldest first (empty if {@code maxHistory == 0}).
     * Returns a snapshot — modifying the returned array does not affect history.
     */
    public Pose3d[] getPoseHistory() {
        return poseHistory.toArray(new Pose3d[0]);
    }

    /**
     * Ordered list of past linear velocities, oldest first.
     */
    public Translation3d[] getVelocityHistory() {
        return velocityHistory.toArray(new Translation3d[0]);
    }

    /**
     * Whether the body is nearly stationary (speed below threshold).
     */
    public boolean isAtRest(double thresholdMetersPerSecond) {
        return getSpeed() < thresholdMetersPerSecond;
    }

    /** Reset accumulated distance and clear history. */
    public void reset() {
        totalDistance = 0;
        poseHistory.clear();
        velocityHistory.clear();
        latestPose = target.getPose();
        latestVelocity = target.getLinearVelocity();
    }
}
