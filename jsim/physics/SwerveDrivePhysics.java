package jsim.physics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Distance;

import java.util.ArrayList;
import java.util.List;

/**
 * Central manager for physics simulation, encapsulating WPILib Kinematics, 
 * Odometry, and environmental collision layers.
 */
public class SwerveDrivePhysics {

    // Internal references for single-source access
    private static SwerveDriveKinematics kinematics;
    private static SwerveDriveOdometry odometry;

    private final Translation2d robotDimensions; // Half-length (X) and half-width (Y) including bumpers
    private final List<PhysicsLayer> layers = new ArrayList<>();

    private Pose2d currentPose = new Pose2d();
    private ChassisSpeeds currentPhysicalSpeeds = new ChassisSpeeds();

    /**
     * Constructs SwerveDrivePhysics and initializes internal Kinematics and Odometry.
     *
     * @param moduleLocations Locations of swerve modules relative to robot center.
     * @param lengthWithBumpers Total robot length (X-axis) including bumpers.
     * @param widthWithBumpers Total robot width (Y-axis) including bumpers.
     * @param initialPose Starting pose on the field.
     * @param initialPositions Initial module wheel positions.
     */
    public SwerveDrivePhysics(
            Translation2d[] moduleLocations,
            Distance lengthWithBumpers,
            Distance widthWithBumpers,
            Pose2d initialPose,
            SwerveModulePosition[] initialPositions) {

        // Construct WPILib Kinematics & Odometry simultaneously
        kinematics = new SwerveDriveKinematics(moduleLocations);
        odometry = new SwerveDriveOdometry(kinematics, initialPose.getRotation(), initialPositions, initialPose);

        this.currentPose = initialPose;
        this.robotDimensions = new Translation2d(
            lengthWithBumpers.in(edu.wpi.first.units.Units.Meters) / 2.0,
            widthWithBumpers.in(edu.wpi.first.units.Units.Meters) / 2.0
        );
    }

    /** Get the single source of truth for SwerveDriveKinematics. */
    public static SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /** Get the single source of truth for SwerveDriveOdometry. */
    public static SwerveDriveOdometry getOdometry() {
        return odometry;
    }

    /** Attaches a physics processing layer to the pipeline. */
    public SwerveDrivePhysics addLayer(PhysicsLayer layer) {
        layers.add(layer);
        return this;
    }

    /**
     * Updates physics, applies all collision/constraint layers, and advances robot pose.
     *
     * @param inputSpeeds       Desired robot-relative ChassisSpeeds commanded by robot logic.
     * @param currentGyroAngle  Current simulated gyro heading.
     * @param modulePositions   Current simulated module encoder positions.
     * @param dtSeconds         Time delta since last update cycle (typically 0.02s).
     * @return                  Calculated physical state containing pose and physical speeds.
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

        // 2. Integrate pose step based on physics-adjusted robot-relative chassis speeds
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

    public Pose2d getPose() {
        return currentPose;
    }

    public ChassisSpeeds getPhysicalSpeeds() {
        return currentPhysicalSpeeds;
    }

    /** Data record returning output physics state. */
    public record PhysicsState(Pose2d pose, ChassisSpeeds physicalSpeeds) {}
}