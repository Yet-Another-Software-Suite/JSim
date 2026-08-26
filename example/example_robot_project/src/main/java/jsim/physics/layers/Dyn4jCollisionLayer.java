package jsim.physics.layers;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Mass;

import org.dyn4j.collision.narrowphase.Penetration;
import org.dyn4j.collision.narrowphase.Sat;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.World;

/**
 * Rigid-body collision physics layer powered by the dyn4j 2D physics engine.
 *
 * <p>Simulates wall impacts, static field structures, and bumper contact by resolving
 * impulse dynamics and translating the resulting velocity vectors back to robot-relative space.
 *
 * @see PhysicsLayer
 * @see FieldLayout
 */
public class Dyn4jCollisionLayer implements PhysicsLayer {

  /**
   * Fallback loop step period in seconds (20ms / 50Hz standard robot loop), used by
   * {@link jsim.physics.SwerveDrivePhysics} for the very first simulation step before a real
   * elapsed time is available.
   */
  public static final double DEFAULT_DT_SECONDS = Milliseconds.of(20).in(Seconds);

  /**
   * Default sentinel constant indicating that rotational inertia should be automatically calculated.
   */
  public static final double AUTO_INERTIA = -1.0;

  /**
   * Default offset vector placing the center of mass at the geometric origin (0, 0) of the chassis.
   */
  public static final Vector2 DEFAULT_CENTER_OF_MASS = new Vector2(0, 0);

  /**
   * Number of relaxation passes {@link #resolveResidualPenetration()} runs each step to converge
   * multiple simultaneous contacts (e.g. a corner wedged into two obstacles at once), where pushing
   * the robot out of one can reintroduce a small overlap with another.
   */
  private static final int PENETRATION_RESOLUTION_ITERATIONS = 4;

  /**
   * Allowed residual overlap, in meters, below which {@link #resolveResidualPenetration()} leaves
   * a contact alone. Matches dyn4j's own default linear slop so this pass doesn't fight the
   * engine's contact solver over sub-millimeter jitter every frame.
   */
  private static final double PENETRATION_SLOP_METERS = 0.0005;

  private final World<Body> world;
  private final Body robotBody;
  private final Mass mass;
  private final FieldLayout fieldLayout;
  private final Sat narrowphase = new Sat();

  /** Custom moment of inertia override in kg·m². A non-positive value indicates auto-calculation. */
  private final double explicitInertia;

  /** Custom center-of-mass offset vector relative to the geometric center in meters. */
  private final Vector2 explicitCenterOfMass;

  private boolean initialized = false;

  /**
   * Constructs a collision layer using <b>automatic mass properties (default)</b>.
   *
   * @param mass Total robot mass including bumpers and battery.
   * @param fieldLayout Static environment geometry to inject into the simulation world.
   */
  public Dyn4jCollisionLayer(Mass mass, FieldLayout fieldLayout) {
    this(mass, AUTO_INERTIA, DEFAULT_CENTER_OF_MASS, fieldLayout);
  }

  /**
   * Constructs a collision layer with an optional custom rotational inertia override.
   *
   * @param mass Total robot mass.
   * @param customInertiaKgM2 Custom moment of inertia in kg·m². Pass {@link #AUTO_INERTIA}
   *                          or any non-positive value to retain automatic calculation.
   * @param fieldLayout Static environment geometry to inject into the simulation world.
   */
  public Dyn4jCollisionLayer(Mass mass, double customInertiaKgM2, FieldLayout fieldLayout) {
    this(mass, customInertiaKgM2, DEFAULT_CENTER_OF_MASS, fieldLayout);
  }

  /**
   * Constructs a collision layer with full manual overrides for physical mass properties.
   *
   * @param mass Total robot mass.
   * @param customInertiaKgM2 Custom moment of inertia in kg·m².
   * @param centerOfMass Offset vector of center of mass relative to robot center in meters.
   * @param fieldLayout Static environment geometry to inject into the simulation world.
   */
  public Dyn4jCollisionLayer(
      Mass mass,
      double customInertiaKgM2,
      Vector2 centerOfMass,
      FieldLayout fieldLayout) {
    this.mass = mass;
    this.explicitInertia = customInertiaKgM2;
    this.explicitCenterOfMass = (centerOfMass != null) ? centerOfMass : DEFAULT_CENTER_OF_MASS;
    this.fieldLayout = fieldLayout;

    this.world = new World<>();
    this.world.setGravity(World.ZERO_GRAVITY);
    this.robotBody = new Body();
  }

  /**
   * Processes input speeds against environmental wall bounds and obstacle physics.
   *
   * @param currentPose Current ground-truth field position of the robot.
   * @param inputSpeeds Desired robot-relative chassis speeds before collision evaluation.
   * @param robotDimensions Half-length (X) and half-width (Y) bumper dimensions in meters.
   * @param dtSeconds Time elapsed since the previous update step, in seconds.
   * @return Physics-constrained {@link ChassisSpeeds} after resolving rigid-body collisions.
   */
  @Override
  public ChassisSpeeds process(
      Pose2d currentPose, ChassisSpeeds inputSpeeds, Translation2d robotDimensions, double dtSeconds) {
    if (!initialized) {
      initEnvironment(robotDimensions);
      initialized = true;
    }

    if (dtSeconds <= 0) {
      return inputSpeeds;
    }

    // 1. Sync dyn4j body pose with WPILib ground-truth pose
    robotBody.getTransform().setTranslation(currentPose.getX(), currentPose.getY());
    robotBody.getTransform().setRotation(currentPose.getRotation().getRadians());

    // 2. Convert robot-relative chassis speeds to field-relative linear vectors
    ChassisSpeeds fieldRelSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(inputSpeeds, currentPose.getRotation());

    robotBody.setLinearVelocity(new Vector2(fieldRelSpeeds.vxMetersPerSecond, fieldRelSpeeds.vyMetersPerSecond));
    robotBody.setAngularVelocity(fieldRelSpeeds.omegaRadiansPerSecond);

    // 3. Advance physics engine by exactly one step of the caller's real elapsed step period.
    world.update(dtSeconds, 0.00001);

    // 4. dyn4j's discrete narrow-phase can leave a small residual overlap after a single step
    resolveResidualPenetration();

    // 5. Derive the effective velocity from dyn4j's
    Vector2 velocity = robotBody.getLinearVelocity();
    ChassisSpeeds postFieldSpeeds = new ChassisSpeeds(velocity.x, velocity.y, robotBody.getAngularVelocity());
    return ChassisSpeeds.fromFieldRelativeSpeeds(postFieldSpeeds, currentPose.getRotation());
  }

  /**
   * Explicitly zeroes out any overlap left between the robot and any other body after
   * {@link org.dyn4j.world.World#update(double, double)} has already run its own (only partial)
   * contact correction pass, so a single step's residual penetration never has anything to
   * accumulate from frame to frame.
   *
   * <p>For each overlapping fixture pair, the robot is pushed directly out along the SAT-reported
   * minimum-translation-vector normal by the full remaining depth (down to {@link
   * #PENETRATION_SLOP_METERS}), and any velocity component still driving further into that
   * obstacle is zeroed so the same overlap doesn't simply reappear next step. Runs several
   * relaxation passes since resolving one contact can reintroduce a small overlap with another
   * (e.g. a corner wedged into two obstacles at once).
   */
  private void resolveResidualPenetration() {
    Transform robotTransform = robotBody.getTransform();
    Penetration penetration = new Penetration();

    for (int iteration = 0; iteration < PENETRATION_RESOLUTION_ITERATIONS; iteration++) {
      boolean anyCorrected = false;

      for (int i = 0; i < world.getBodyCount(); i++) {
        Body other = world.getBody(i);
        if (other == robotBody) {
          continue;
        }

        for (BodyFixture robotFixture : robotBody.getFixtures()) {
          for (BodyFixture otherFixture : other.getFixtures()) {
            penetration.clear();
            boolean overlapping = narrowphase.detect(
                robotFixture.getShape(), robotTransform,
                otherFixture.getShape(), other.getTransform(),
                penetration);
            if (!overlapping || penetration.getDepth() <= PENETRATION_SLOP_METERS) {
              continue;
            }

            // Sat.detect's normal always points from the first shape (the robot, here) toward the
            // second (the other body), so the robot must move the OPPOSITE way to separate from it.
            Vector2 normal = penetration.getNormal();
            double pushOutDistance = penetration.getDepth() - PENETRATION_SLOP_METERS;
            robotTransform.translate(-normal.x * pushOutDistance, -normal.y * pushOutDistance);

            Vector2 velocity = robotBody.getLinearVelocity();
            double velocityIntoObstacle = velocity.dot(normal);
            if (velocityIntoObstacle > 0) {
              robotBody.setLinearVelocity(velocity.difference(normal.product(velocityIntoObstacle)));
            }
            anyCorrected = true;
          }
        }
      }

      if (!anyCorrected) {
        break;
      }
    }
  }

  /**
   * Immediately re-syncs the dyn4j body's transform and velocity to {@code pose} and
   * {@code robotRelativeSpeeds}, discarding any momentum or contact state accumulated from prior
   * {@link #process} calls (e.g. residual bounce velocity from a collision).
   *
   * <p>If {@link #process} hasn't run yet, this is a no-op: {@link #initEnvironment} runs on the
   * first {@code process} call and syncs from the caller's current pose/speeds at that point
   * anyway, so there is no body to reset yet.
   *
   * @param pose Field-relative pose to reset to.
   * @param robotRelativeSpeeds Robot-relative chassis speeds to assume immediately after reset.
   */
  @Override
  public void reset(Pose2d pose, ChassisSpeeds robotRelativeSpeeds) {
    if (!initialized) {
      return;
    }

    robotBody.getTransform().setTranslation(pose.getX(), pose.getY());
    robotBody.getTransform().setRotation(pose.getRotation().getRadians());

    ChassisSpeeds fieldRelSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, pose.getRotation());
    robotBody.setLinearVelocity(new Vector2(fieldRelSpeeds.vxMetersPerSecond, fieldRelSpeeds.vyMetersPerSecond));
    robotBody.setAngularVelocity(fieldRelSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Initializes robot bumper geometry, configures baseline automatic mass calculation,
   * applies explicit parameter overrides if non-default values were supplied, and injects field layout obstacles.
   *
   * @param robotDimensions Half-length (X) and half-width (Y) bumper footprint dimensions in meters.
   */
  private void initEnvironment(Translation2d robotDimensions) {
    double length = robotDimensions.getX() * 2.0;
    double width = robotDimensions.getY() * 2.0;
    double totalMassKg = mass.in(Kilograms);

    BodyFixture fixture = robotBody.addFixture(
        Geometry.createRectangle(length, width)
    );
    fixture.setFriction(0.2);
    fixture.setRestitution(0.1);

    // 1. AUTOMATIC MASS CALCULATION (DEFAULT BASELINE)
    double footprintArea = length * width;
    fixture.setDensity(totalMassKg / footprintArea);
    robotBody.setMass(MassType.NORMAL);

    // 2. MANUAL OVERRIDE EVALUATION
    boolean hasCustomInertia = explicitInertia > 0.0;
    boolean hasCustomCoM = !explicitCenterOfMass.equals(DEFAULT_CENTER_OF_MASS);

    if (hasCustomInertia || hasCustomCoM) {
      double effectiveInertia = hasCustomInertia ? explicitInertia : robotBody.getMass().getInertia();
      robotBody.setMass(new org.dyn4j.geometry.Mass(explicitCenterOfMass, totalMassKg, effectiveInertia));
    }

    world.addBody(robotBody);

    // Inject field obstacles into the physics world
    if (fieldLayout != null) {
      fieldLayout.populateWorld(world);
    }
  }
}