package jsim.physics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Distance;
import yams.mechanisms.swerve.SwerveDrive;

import java.util.ArrayList;
import java.util.List;

/**
 * Central orchestrator and manager for swerve drivetrain physics simulation within JSIM.
 * * <p>Standard WPILib simulation models ideal kinematic movement where commanded speeds directly
 * translate to ground position. {@code SwerveDrivePhysics} bridges the gap between ideal kinematics
 * and physical field environments by providing:
 * * <ul>
 * <li><b>Single Source Kinematics & Odometry:</b> Instantiates and encapsulates WPILib's
 * {@link SwerveDriveKinematics} and {@link SwerveDriveOdometry} to prevent state synchronization
 * mismatches across subsystem classes.</li>
 * <li><b>Modular Layer Pipeline:</b> Evaluates physical environmental constraints (such as rigid wall collisions,
 * skid friction, or game piece interaction) through sequential {@link PhysicsLayer} passes.</li>
 * <li><b>Simulated Ground Truth vs Sensor Odometry:</b> Tracks true physical field position
 * ({@code currentPose}) after accounting for collisions, while simultaneously feeding wheel encoder
 * data into internal odometry for sensor estimation.</li>
 * </ul>
 * * <h3>Usage Example (YAMS Drivetrain Integration)</h3>
 * <pre>{@code
 * public class SwerveSubsystem extends SubsystemBase {
 * private final SwerveDrive drive;
 * private final SwerveDrivePhysics physicsSim;
 * * public SwerveSubsystem() {
 * // ... initialize YAMS drive ...
 * physicsSim = new SwerveDrivePhysics(drive)
 * .addLayer(new Dyn4jCollisionLayer(
 * Kilograms.of(50.0),
 * FieldLayout.LOAD_2026_FIELD
 * ));
 * }
 * * @Override
 * public void simulationPeriodic() {
 * drive.simIterate();
 * physicsSim.update(); // Steps physical world and updates pose ground truth
 * }
 * }
 * }</pre>
 * * @see PhysicsLayer
 * @see PhysicsState
 */
public class SwerveDrivePhysics {

    /** Single-source static instance of WPILib kinematics derived during instantiation. */
    private static SwerveDriveKinematics kinematics;

    /** Single-source static instance of WPILib odometry derived during instantiation. */
    private static SwerveDriveOdometry odometry;

    /** Half-length (X) and half-width (Y) bumper footprint dimensions in meters. */
    private final Translation2d robotDimensions;

    /** Sequential processing pipeline containing active physics layers. */
    private final List<PhysicsLayer> layers = new ArrayList<>();

    /** Reference to parent YAMS drivebase instance, or {@code null} if using raw WPILib constructor. */
    private final SwerveDrive yamsDrive;

    /** True physical ground-truth position on the field (incorporating wall bounds & forces). */
    private Pose2d currentPose = new Pose2d();

    /** Current physical robot-relative chassis velocity after layer transformation. */
    private ChassisSpeeds currentPhysicalSpeeds = new ChassisSpeeds();

    /**
     * Constructs a {@code SwerveDrivePhysics} manager bound to a YAMS {@link SwerveDrive} chassis.
     * * <p>This constructor automatically extracts kinematic configurations, module locations,
     * initial starting pose, and robot outer bumper dimensions directly from the YAMS abstraction,
     * eliminating duplicated geometry constants.
     *
     * @param drive The parent YAMS {@link SwerveDrive} subsystem instance. Must not be {@code null}.
     */
    public SwerveDrivePhysics(SwerveDrive drive) {
        this.yamsDrive = drive;
        
        kinematics = drive.getKinematics();
        odometry = drive.getOdometry();
        this.currentPose = drive.getPose();
        
        this.robotDimensions = drive.getRobotDimensions();
    }

    /**
     * Constructs a standalone {@code SwerveDrivePhysics} manager using raw WPILib structures
     * without a YAMS dependency wrapper.
     *
     * @param moduleLocations    Array of module positions relative to robot center (in WPILib frame: +X forward, +Y left).
     * @param lengthWithBumpers Total robot bumper-to-bumper length along the X-axis.
     * @param widthWithBumpers  Total robot bumper-to-bumper width along the Y-axis.
     * @param initialPose        Starting pose of the robot on the field.
     * @param initialPositions   Initial module wheel positions (distances and angles).
     */
    public SwerveDrivePhysics(
            Translation2d[] moduleLocations,
            Distance lengthWithBumpers,
            Distance widthWithBumpers,
            Pose2d initialPose,
            SwerveModulePosition[] initialPositions) {

        this.yamsDrive = null;
        kinematics = new SwerveDriveKinematics(moduleLocations);
        odometry = new SwerveDriveOdometry(kinematics, initialPose.getRotation(), initialPositions, initialPose);

        this.currentPose = initialPose;
        this.robotDimensions = new Translation2d(
            lengthWithBumpers.in(edu.wpi.first.units.Units.Meters) / 2.0,
            widthWithBumpers.in(edu.wpi.first.units.Units.Meters) / 2.0
        );
    }

    /**
     * Returns the single-source-of-truth {@link SwerveDriveKinematics} instance created during initialization.
     *
     * @return The static kinematics object shared across the drivetrain.
     */
    public static SwerveDriveKinematics getKinematics() { 
        return kinematics; 
    }

    /**
     * Returns the single-source-of-truth {@link SwerveDriveOdometry} instance created during initialization.
     *
     * @return The static odometry object shared across the drivetrain.
     */
    public static SwerveDriveOdometry getOdometry() { 
        return odometry; 
    }

    /**
     * Registers a new functional {@link PhysicsLayer} into the execution pipeline.
     * * <p>Layers process sequentially in the order they are added. For example, adding a floor friction layer
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
     * Advances the physics simulation by a single standard loop cycle (20ms) using parameters
     * automatically queried from the bound YAMS {@link SwerveDrive}.
     * * <p>This convenience method is designed for use inside {@code Subsystem.simulationPeriodic()}
     * when using YAMS drivetrain abstractions.
     *
     * @return A {@link PhysicsState} record containing updated ground-truth pose and physics-adjusted chassis speeds.
     * @throws IllegalStateException If called on an instance initialized without a YAMS {@link SwerveDrive}.
     */
    public PhysicsState update() {
        if (yamsDrive == null) {
            throw new IllegalStateException("Cannot call update() without parameters unless instantiated with a YAMS SwerveDrive!");
        }
        return update(
            yamsDrive.getDesiredChassisSpeeds(),
            yamsDrive.getHeading(),
            yamsDrive.getModulePositions(),
            0.020
        );
    }

    /**
     * Advances the physics simulation step-by-step using explicit input parameters.
     * * <p>Execution follows a three-stage pipeline:
     * <ol>
     * <li><b>Layer Processing:</b> Passes {@code inputSpeeds} sequentially through all registered
     * {@link PhysicsLayer} instances to produce physics-constrained speeds.</li>
     * <li><b>Ground-Truth Integration:</b> Integrates the constrained chassis speeds over {@code dtSeconds}
     * to step the true physical field {@link Pose2d}.</li>
     * <li><b>Odometry Update:</b> Updates the internal WPILib odometry tracker using simulated encoder feedback.</li>
     * </ol>
     *
     * @param inputSpeeds       Desired robot-relative chassis speeds commanded by robot logic.
     * @param currentGyroAngle  Current simulated gyro orientation reading.
     * @param modulePositions   Current simulated swerve module positions (wheel distance and module angle).
     * @param dtSeconds         Time step duration in seconds (typically {@code 0.020} for 50Hz periodic loops).
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
            processedSpeeds = layer.process(currentPose, processedSpeeds, robotDimensions);
        }

        this.currentPhysicalSpeeds = processedSpeeds;

        // 2. Step integration: Transform robot-relative delta velocities into field-relative displacement
        double deltaX = processedSpeeds.vxMetersPerSecond * dtSeconds;
        double deltaY = processedSpeeds.vyMetersPerSecond * dtSeconds;
        double deltaTheta = processedSpeeds.omegaRadiansPerSecond * dtSeconds;

        Translation2d translationStep = new Translation2d(deltaX, deltaY).rotateBy(currentPose.getRotation());
        currentPose = new Pose2d(
            currentPose.getTranslation().plus(translationStep),
            currentPose.getRotation().plus(Rotation2d.fromRadians(deltaTheta))
        );

        // 3. Update internal single-source Odometry
        odometry.update(currentGyroAngle, modulePositions);

        return new PhysicsState(currentPose, currentPhysicalSpeeds);
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
     * @param pose           The calculated ground-truth field pose following physics step integration.
     * @param physicalSpeeds The physics-adjusted robot-relative speeds after pipeline evaluation.
     */
    public record PhysicsState(Pose2d pose, ChassisSpeeds physicalSpeeds) {}
}