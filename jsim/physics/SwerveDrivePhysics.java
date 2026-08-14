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

public class SwerveDrivePhysics {

    private static SwerveDriveKinematics kinematics;
    private static SwerveDriveOdometry odometry;

    private final Translation2d robotDimensions;
    private final List<PhysicsLayer> layers = new ArrayList<>();
    private final SwerveDrive yamsDrive; // Reference to YAMS drivebase if provided

    private Pose2d currentPose = new Pose2d();
    private ChassisSpeeds currentPhysicalSpeeds = new ChassisSpeeds();

    /**
     * YAMS Constructor: Automatically pulls kinematics, module locations, and pose from YAMS.
     */
    public SwerveDrivePhysics(SwerveDrive drive) {
        this.yamsDrive = drive;
        
        // Extract kinematics & initial state directly from YAMS
        kinematics = drive.getKinematics();
        odometry = drive.getOdometry();
        this.currentPose = drive.getPose();
        
        // Extract footprint dimensions from module offsets (or default estimates)
        this.robotDimensions = drive.getRobotDimensions();
    }

    /**
     * Standard WPILib Constructor (Without YAMS wrapper).
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

    public static SwerveDriveKinematics getKinematics() { return kinematics; }
    public static SwerveDriveOdometry getOdometry() { return odometry; }

    public SwerveDrivePhysics addLayer(PhysicsLayer layer) {
        layers.add(layer);
        return this;
    }

    /**
     * Zero-argument update call used inside Subsystem.simulationPeriodic() when using YAMS.
     */
    public PhysicsState update() {
        if (yamsDrive == null) {
            throw new IllegalStateException("Cannot call update() without parameters unless instantiated with a YAMS SwerveDrive!");
        }
        return update(
            yamsDrive.getDesiredChassisSpeeds(),
            yamsDrive.getHeading(),
            yamsDrive.getModulePositions(),
            0.020 // Standard WPILib loop period
        );
    }

    /**
     * Explicit update step with custom parameters.
     */
    public PhysicsState update(
            ChassisSpeeds inputSpeeds,
            Rotation2d currentGyroAngle,
            SwerveModulePosition[] modulePositions,
            double dtSeconds) {

        // 1. Process desired speeds through all registered physics layers
        ChassisSpeeds processedSpeeds = inputSpeeds;
        for (PhysicsLayer layer : layers) {
            processedSpeeds = layer.process(currentPose, processedSpeeds, robotDimensions);
        }

        this.currentPhysicalSpeeds = processedSpeeds;

        // 2. Step integration
        double deltaX = processedSpeeds.vxMetersPerSecond * dtSeconds;
        double deltaY = processedSpeeds.vyMetersPerSecond * dtSeconds;
        double deltaTheta = processedSpeeds.omegaRadiansPerSecond * dtSeconds;

        Translation2d translationStep = new Translation2d(deltaX, deltaY).rotateBy(currentPose.getRotation());
        currentPose = new Pose2d(
            currentPose.getTranslation().plus(translationStep),
            currentPose.getRotation().plus(Rotation2d.fromRadians(deltaTheta))
        );

        // 3. Update odometry
        odometry.update(currentGyroAngle, modulePositions);

        return new PhysicsState(currentPose, currentPhysicalSpeeds);
    }

    public Pose2d getPose() { return currentPose; }
    public ChassisSpeeds getPhysicalSpeeds() { return currentPhysicalSpeeds; }

    public record PhysicsState(Pose2d pose, ChassisSpeeds physicalSpeeds) {}
}