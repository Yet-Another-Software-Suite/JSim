package jsim.physics;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import jsim.physics.layers.PhysicsLayer;
import yams.mechanisms.swerve.SwerveDrive;
import java.util.ArrayList;
import java.util.List;

/**
 * Central orchestrator and manager for swerve drivetrain physics simulation within JSIM.
 *
 * <p>Standard WPILib simulation models ideal kinematic movement where commanded speeds directly
 * translate to ground position. {@code SwerveDrivePhysics} bridges the gap between ideal kinematics
 * and physical field environments by providing:
 *
 * <ul>
 * <li><b>Single Source Kinematics & Odometry:</b> Instantiates and encapsulates WPILib's
 * {@link SwerveDriveKinematics} and {@link SwerveDriveOdometry} to prevent state synchronization
 * mismatches across subsystem classes.</li>
 * <li><b>Modular Layer Pipeline:</b> Evaluates physical environmental constraints (such as rigid wall collisions,
 * skid friction, or game piece interaction) through sequential {@link PhysicsLayer} passes.</li>
 * <li><b>Simulated Ground Truth vs Sensor Odometry:</b> Tracks true physical field position
 * ({@code currentPose}) after accounting for collisions, while simultaneously feeding wheel encoder
 * data into internal odometry for sensor estimation.</li>
 * </ul>
 *
 * <h3>Usage Example (YAMS Drivetrain Integration)</h3>
 * <pre>{@code
 * public class SwerveSubsystem extends SubsystemBase {
 * private final SwerveDrive drive;
 * private final SwerveDrivePhysics physicsSim;
 *
 * public SwerveSubsystem() {
 * // ... initialize YAMS drive ...
 * physicsSim = new SwerveDrivePhysics(drive)
 * .addLayer(new Dyn4jCollisionLayer(
 * Kilograms.of(50.0),
 * new Field2026()
 * ));
 * }
 *
 * @Override
 * public void simulationPeriodic() {
 * drive.simIterate();
 * physicsSim.update(); // Steps physical world and updates pose ground truth
 * }
 * }
 * }</pre>
 *
 * @see PhysicsLayer
 * @see PhysicsState
 */
public class SwerveDrivePhysics {

    /**
     * Fallback loop period assumed for the very first {@link #update()} call, before a prior
     * {@link Timer#getFPGATimestamp()} reading exists to diff against.
     */
    private static final double DEFAULT_DT_SECONDS = 0.020;

    /** Single-source final instance of WPILib kinematics derived during instantiation. */
    private final SwerveDriveKinematics kinematics;

    /** Single-source final instance of WPILib odometry derived during instantiation. */
    private final SwerveDrivePoseEstimator odometry;

    /** Half-length (X) and half-width (Y) bumper footprint dimensions in meters. */
    private final Translation2d robotDimensions;

    /** Field2d object for SmartDashboard visualization of the robot's ground-truth pose. */
    private final Field2d field2d;

    /** Sequential processing pipeline containing active physics layers. */
    private final List<PhysicsLayer> layers = new ArrayList<>();

    /** Reference to parent YAMS drivebase instance, or {@code null} if using raw WPILib constructor. */
    private final SwerveDrive yamsDrive;

    /** True physical ground-truth position on the field (incorporating wall bounds & forces). */
    private Pose2d currentPose = new Pose2d();

    /** Current physical robot-relative chassis velocity after layer transformation. */
    private ChassisSpeeds currentPhysicalSpeeds = new ChassisSpeeds();

    /** FPGA timestamp of the previous {@link #update()} call, or a negative value before the first call. */
    private double lastUpdateTimestampSeconds = -1.0;

    /** Most recently supplied gyro angle, cached so {@link #resetOdometry} can re-baseline {@link #odometry}. */
    private Rotation2d lastGyroAngle;

    /** Most recently supplied module positions, cached so {@link #resetOdometry} can re-baseline {@link #odometry}. */
    private SwerveModulePosition[] lastModulePositions;

  /**
   * NetworkTable publisher for pose data.
   */
  private StructPublisher<Pose2d> posePublisher;

    /**
     * Constructs a {@code SwerveDrivePhysics} manager bound to a YAMS {@link SwerveDrive} chassis.
     *
     * <p>This constructor automatically extracts kinematic configurations, module locations,
     * initial starting pose, and robot outer bumper dimensions directly from the YAMS abstraction,
     * eliminating duplicated geometry constants.
     *
     * @param drive The parent YAMS {@link SwerveDrive} subsystem instance. Must not be {@code null}.
     */
    public SwerveDrivePhysics(SwerveDrive drive) {
        this.yamsDrive = drive;
        var ntEntry = NetworkTableInstance.getDefault().getTable("Mechanisms").getSubTable(drive.getName()).getStructTopic("physics", Pose2d.struct);
        posePublisher = ntEntry.publish();
        field2d = drive.getField2d();
        odometry = new SwerveDrivePoseEstimator(drive.getKinematics(), new Rotation2d(drive.getGyroAngle()), drive.getModulePositions(), drive.getConfig().getInitialPose());

        kinematics = drive.getKinematics();
        this.currentPose = drive.getPose();
        this.lastGyroAngle = new Rotation2d(drive.getGyroAngle());
        this.lastModulePositions = drive.getModulePositions();

        // fl, fr, bl, br
        Translation2d fl = drive.getConfig().getModules()[0].getConfig().getLocation().orElseThrow();
        Translation2d br = drive.getConfig().getModules()[3].getConfig().getLocation().orElseThrow();

        this.robotDimensions = new Translation2d(
                (Math.abs(fl.getX()) + Math.abs(br.getX())) / 2.0,
                (Math.abs(fl.getY()) + Math.abs(br.getY())) / 2.0);
    }

    /**
     * Constructs a standalone {@code SwerveDrivePhysics} manager using raw WPILib structures
     * without a YAMS dependency wrapper.
     *
     * @param moduleLocations   Array of module positions relative to robot center (in WPILib frame: +X forward, +Y left).
     * @param lengthWithBumpers Total robot bumper-to-bumper length along the X-axis.
     * @param widthWithBumpers  Total robot bumper-to-bumper width along the Y-axis.
     * @param initialPose       Starting pose of the robot on the field.
     * @param initialPositions  Initial module wheel positions (distances and angles).
     */
    public SwerveDrivePhysics(
            Translation2d[] moduleLocations,
            Distance lengthWithBumpers,
            Distance widthWithBumpers,
            Pose2d initialPose,
            SwerveModulePosition[] initialPositions) {

        this.yamsDrive = null;
        field2d = new Field2d();
        var ntEntry = NetworkTableInstance.getDefault().getTable("Mechanisms").getSubTable("swerve").getStructTopic("physics", Pose2d.struct);
        posePublisher = ntEntry.publish();

        kinematics = new SwerveDriveKinematics(moduleLocations);
        odometry = new SwerveDrivePoseEstimator(kinematics, initialPose.getRotation(), initialPositions, initialPose);

        this.currentPose = initialPose;
        this.lastGyroAngle = initialPose.getRotation();
        this.lastModulePositions = initialPositions;
        this.robotDimensions = new Translation2d(
                lengthWithBumpers.in(edu.wpi.first.units.Units.Meters) / 2.0,
                widthWithBumpers.in(edu.wpi.first.units.Units.Meters) / 2.0);
    }

    /**
     * Returns the single-source-of-truth {@link SwerveDriveKinematics} instance created during initialization.
     *
     * @return The final kinematics object shared across the drivetrain.
     */
    public final SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Returns the single-source-of-truth {@link SwerveDriveOdometry} instance created during initialization.
     *
     * @return The final odometry object shared across the drivetrain.
     */
    public final SwerveDrivePoseEstimator getOdometry() {
        return odometry;
    }

    /**
     * Registers a new functional {@link PhysicsLayer} into the execution pipeline.
     *
     * <p>Layers process sequentially in the order they are added. For example, adding a floor friction layer
     * before a wall collision layer ensures speed reductions due to carpet drag are evaluated before collision
     * impulse dynamics.
     *
     * @param layer The {@link PhysicsLayer} instance to append to the pipeline.
     * @return This {@code SwerveDrivePhysics} instance to support fluent method chaining.
     */
    public SwerveDrivePhysics addLayer(PhysicsLayer layer) {
        layers.add(layer);
        return this;
    }

    /**
     * Advances the physics simulation by a single loop cycle using parameters automatically
     * queried from the bound YAMS {@link SwerveDrive}, with the step duration derived from the
     * real elapsed time since the previous call via {@link Timer#getFPGATimestamp()} (falling
     * back to {@link #DEFAULT_DT_SECONDS} on the first call).
     *
     * <p>Uses {@link SwerveDrive#getRobotRelativeSpeed()} (the drive's actual measured module
     * velocities) rather than {@link SwerveDrive#getDesiredChassisSpeeds()} (the raw commanded
     * setpoint) as the ground-truth input, so this pose tracks the drive's real simulated motion
     * -- including any PID/feedforward ramp-up or current-limit lag -- instead of assuming the
     * commanded speed is achieved instantly.
     *
     * <p>This convenience method is designed for use inside {@code Subsystem.simulationPeriodic()}
     * when using YAMS drivetrain abstractions.
     *
     * @return A {@link PhysicsState} record containing updated ground-truth pose and physics-adjusted chassis speeds.
     * @throws IllegalStateException If called on an instance initialized without a YAMS {@link SwerveDrive}.
     */
    public PhysicsState update() {
        if (yamsDrive == null) {
            throw new IllegalStateException(
                    "Cannot call update() without parameters unless instantiated with a YAMS SwerveDrive!");
        }
        double now = Timer.getFPGATimestamp();
        double dtSeconds = (lastUpdateTimestampSeconds < 0)
                ? DEFAULT_DT_SECONDS
                : now - lastUpdateTimestampSeconds;
        lastUpdateTimestampSeconds = now;

        return update(
                yamsDrive.getRobotRelativeSpeed(),
                new Rotation2d(yamsDrive.getGyroAngle()),
                yamsDrive.getModulePositions(),
                dtSeconds);
    }

    /**
     * Advances the physics simulation step-by-step using explicit input parameters.
     *
     * <p>Execution follows a three-stage pipeline:
     * <ol>
     * <li><b>Layer Processing:</b> Passes {@code inputSpeeds} sequentially through all registered
     * {@link PhysicsLayer} instances to produce physics-constrained speeds.</li>
     * <li><b>Ground-Truth Integration:</b> Integrates the constrained chassis speeds over {@code dtSeconds}
     * using exponential twist integration to step the true physical field {@link Pose2d}.</li>
     * <li><b>Odometry Update:</b> Updates the internal WPILib odometry tracker using simulated encoder feedback.</li>
     * </ol>
     *
     * @param inputSpeeds      Desired robot-relative chassis speeds commanded by robot logic.
     * @param currentGyroAngle Current simulated gyro orientation reading.
     * @param modulePositions  Current simulated swerve module positions (wheel distance and module angle).
     * @param dtSeconds        Time step duration in seconds (typically {@code 0.020} for 50Hz periodic loops).
     * @return A {@link PhysicsState} record containing the calculated physical pose and modified speeds.
     */
    public PhysicsState update(
            ChassisSpeeds inputSpeeds,
            Rotation2d currentGyroAngle,
            SwerveModulePosition[] modulePositions,
            double dtSeconds) {

        // 1. Pipeline processing: Pass desired speeds sequentially through all layers
        ChassisSpeeds processedSpeeds = inputSpeeds;
        for (PhysicsLayer layer : layers) {
            processedSpeeds = layer.process(currentPose, processedSpeeds, robotDimensions, dtSeconds);
        }

        this.currentPhysicalSpeeds = processedSpeeds;

        // 2. Step integration: Exponential arc step (eliminates rotational/translational drift)
        //
        // Heading is taken directly from currentGyroAngle rather than integrating
        // processedSpeeds.omegaRadiansPerSecond. A real swerve chassis's orientation is governed by
        // its own drivetrain (independently steerable, gripped wheels resisting unwanted rotation)
        // and read from a real gyro -- it isn't a passive box that spins freely from contact torque.
        // Letting collision-layer torque feed back into the ground-truth heading creates a runaway
        // loop: an off-center contact (e.g. a corner hit) imparts angular velocity within one dyn4j
        // step, that rotates currentPose, which then reinterprets the next frame's *robot-relative*
        // command along a new field direction, compounding every frame until the robot appears to
        // slide/orbit around the obstacle instead of stopping against it.
        Twist2d twist = new Twist2d(
                processedSpeeds.vxMetersPerSecond * dtSeconds,
                processedSpeeds.vyMetersPerSecond * dtSeconds,
                processedSpeeds.omegaRadiansPerSecond * dtSeconds);
        Pose2d integratedPose = currentPose.exp(twist);
        currentPose = new Pose2d(integratedPose.getTranslation(), currentGyroAngle);

        // 3. Update internal single-source Odometry
        odometry.update(currentGyroAngle, modulePositions);
        this.lastGyroAngle = currentGyroAngle;
        this.lastModulePositions = modulePositions;

        // 4. Publish physics pose to SmartDashboard
        field2d.getObject("jsim").setPose(currentPose);
        posePublisher.accept(currentPose);
//        field2d.getObject("odometry").setPose(odometry.getEstimatedPosition());

        return new PhysicsState(currentPose, currentPhysicalSpeeds);
    }

    /**
     * Resets the physics simulation to a stationary state at {@code pose}, as if the robot had
     * just been placed there at rest (e.g. re-homing to a known field position). Equivalent to
     * {@code resetOdometry(pose, new ChassisSpeeds())}.
     *
     * @param pose The field-relative pose to reset the ground-truth position and odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        resetOdometry(pose, new ChassisSpeeds());
    }

    /**
     * Resets the physics simulation so the robot is at {@code pose} moving at
     * {@code robotRelativeSpeeds}, discarding any prior ground-truth pose, physical velocity, and
     * per-layer simulation state (e.g. a collision layer's residual bounce momentum).
     *
     * <p>Every registered {@link PhysicsLayer} is notified via {@link PhysicsLayer#reset} so
     * layers that carry their own internal state (such as {@code Dyn4jCollisionLayer}'s rigid
     * body) re-sync to the new pose/speeds immediately rather than waiting for the next
     * {@link #update} call.
     *
     * @param pose                 The field-relative pose to reset the ground-truth position and odometry to.
     * @param robotRelativeSpeeds  The robot-relative chassis speeds to assume immediately after
     *                             reset. Callers already tracking a desired speed (e.g. from
     *                             {@code SwerveDrive#getDesiredChassisSpeeds()}) can pass that
     *                             directly.
     */
    public void resetOdometry(Pose2d pose, ChassisSpeeds robotRelativeSpeeds) {
        this.currentPose = pose;
        this.currentPhysicalSpeeds = robotRelativeSpeeds;
        this.lastUpdateTimestampSeconds = -1.0;

        for (PhysicsLayer layer : layers) {
            layer.reset(pose, robotRelativeSpeeds);
        }

        odometry.resetPosition(lastGyroAngle, lastModulePositions, pose);
        field2d.getObject("jsim").setPose(pose);
        posePublisher.accept(pose);
//        field2d.getObject("odometry").setPose(odometry.getEstimatedPosition());
    }

    /**
     * Gets the current true physical pose of the robot on the field, incorporating all wall collisions
     * and layer transformations.
     *
     * @return The physical ground-truth {@link Pose2d}.
     */
    public Pose2d getPose() {
        return currentPose;
    }

    /**
     * Gets the current physics-adjusted robot-relative chassis speeds resulting from the latest update step.
     *
     * @return The constrained {@link ChassisSpeeds}.
     */
    public ChassisSpeeds getPhysicalSpeeds() {
        return currentPhysicalSpeeds;
    }

    /**
     * Immutable data container holding the output results of a physics simulation update step.
     *
     * @param pose          The calculated ground-truth field pose following physics step integration.
     * @param physicalSpeeds The physics-adjusted robot-relative speeds after pipeline evaluation.
     */
    public record PhysicsState(Pose2d pose, ChassisSpeeds physicalSpeeds) {}
}