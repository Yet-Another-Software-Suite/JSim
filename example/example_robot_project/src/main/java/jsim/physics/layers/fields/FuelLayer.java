package jsim.physics.layers.fields;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import jsim.physics.layers.FieldLayout.Element;
import jsim.physics.layers.PhysicsLayer;
import jsim.physics.layers.gamepieces.Fuel2026;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Three-dimensional FUEL simulation for the 2026 REBUILT game, as a {@link PhysicsLayer}.
 *
 * <p>Every FUEL ball is a point mass with a 3D position and velocity (see {@link Fuel2026}),
 * integrated forwards under gravity and (optionally) aerodynamic drag, then collided against the
 * carpet, the {@link Field2026} structures, the other balls, and the robot. Balls are 6in spheres
 * that fly, bounce, roll up BUMPS, rattle off TRENCH BARS, come to rest on the carpet, fall into a
 * HUB and get dispersed back onto the FIELD -- none of which a 2D rigid-body engine can express,
 * so this layer runs its own solver rather than dyn4j.
 *
 * <p>All field geometry comes from {@link Field2026#getGamePieceObstacles()} and the HUB/DEPOT
 * accessors alongside it, so there is exactly one description of the field: change a structure
 * there and both the drivetrain collision layer and this one follow.
 *
 * <h3>Behaviour as a layer</h3>
 *
 * <p>{@link #process} returns the chassis speeds it was handed, unmodified: a 0.45lb ball has no
 * meaningful effect on a 100lb+ robot's velocity, so this layer reads the robot's motion (to push
 * balls out of the way and to feed the intakes) without constraining it. That makes it safe to add
 * anywhere in the pipeline, though adding it last means it sees the post-collision pose:
 *
 * <pre>{@code
 * var field = new Field2026();
 * fuel = new FuelLayer(field).withBumperHeight(Inches.of(8));
 * fuel.registerIntake(
 *     new Transform3d(Inches.of(12), Inches.of(-9), Inches.zero(), Rotation3d.kZero),
 *     new Transform3d(Inches.of(20), Inches.of(9), Inches.of(6), Rotation3d.kZero),
 *     intakeSubsystem::isRunning,
 *     intakeSubsystem::onFuelAcquired);
 *
 * physicsSim = new SwerveDrivePhysics(drive)
 *     .addLayer(new Dyn4jCollisionLayer(SwerveConstants.kRobotMass, field))
 *     .addLayer(fuel);
 * }</pre>
 *
 * <p>The physical model here follows
 * <a href="https://github.com/hammerheads5000/FuelSim">FuelSim</a>.
 *
 * @see Field2026
 * @see Fuel2026
 */
public class FuelLayer implements PhysicsLayer {

  /** Mass of one FUEL ball. */
  public static final Mass FUEL_MASS = Pounds.of(0.448);

  /** Diameter of one FUEL ball. */
  public static final Distance FUEL_DIAMETER = Meters.of(0.15);

  /** Radius of one FUEL ball, in meters. */
  private static final double FUEL_RADIUS = FUEL_DIAMETER.in(Meters) / 2.0;

  /** Mass of one FUEL ball, in kilograms. */
  private static final double FUEL_MASS_KG = FUEL_MASS.in(Kilograms);

  /** Downwards acceleration due to gravity, in m/s^2. */
  private static final double GRAVITY = -9.81;

  /** Density of dry air at room temperature, in kg/m^3. */
  private static final double AIR_DENSITY = 1.2041;

  /** Drag coefficient of a smooth sphere, dimensionless. */
  private static final double DRAG_COEFFICIENT = 0.47;

  /** Constant part of the drag force, i.e. {@code 0.5 * rho * Cd * A}. */
  private static final double DRAG_FORCE_FACTOR =
      0.5 * AIR_DENSITY * DRAG_COEFFICIENT * Math.PI * FUEL_RADIUS * FUEL_RADIUS;

  /** Coefficient of restitution between FUEL and a field structure. */
  private static final double FIELD_COR = Math.sqrt(22.0 / 51.5);

  /** Coefficient of restitution between two FUEL balls. */
  private static final double FUEL_COR = 0.5;

  /** Coefficient of restitution between FUEL and a HUB net, which absorbs most of the impact. */
  private static final double NET_COR = 0.2;

  /** Coefficient of restitution between FUEL and a robot bumper. */
  private static final double ROBOT_COR = 0.1;

  /** Proportion of horizontal velocity a resting ball loses to rolling friction per second. */
  private static final double ROLLING_FRICTION = 0.1;

  /** Vertical speed, in m/s, below which a ball touching a surface settles onto it. */
  private static final double REST_SPEED = 0.05;

  /**
   * Gap to a surface, in meters, that still counts as touching it for the purpose of settling a
   * nearly-stopped ball onto it.
   *
   * <p>Discrete time steps leave a ball that has finished bouncing hovering a fraction of a
   * millimetre above whatever it landed on: it is pulled down by gravity, bounced back up on the
   * step that overlaps, and never samples the exact instant of contact. Letting a slow ball within
   * this gap of a flat surface snap down onto it collapses that jitter into resting contact.
   * Detection is only widened for surfaces a ball can rest on -- a ball is never pushed out of a
   * wall it isn't actually touching.
   */
  private static final double CONTACT_SKIN = 0.005;

  /**
   * Minimum Z component of a contact normal for that contact to count as holding the ball up.
   * {@code cos(45 deg)} and above, i.e. a surface flat enough to rest on rather than roll off.
   */
  private static final double SUPPORT_NORMAL_Z = 0.7;

  /** Edge length of one broadphase grid cell, in meters. Comfortably above one ball diameter. */
  private static final double CELL_SIZE = 0.25;

  /** Default number of physics iterations per {@link #process} call. */
  private static final int DEFAULT_SUBTICKS = 5;

  /** Default rate at which ball positions are published to NetworkTables, in Hz. */
  private static final double DEFAULT_LOGGING_FREQUENCY_HZ = 10.0;

  /** Default height of the top of the robot's bumpers above the carpet. */
  private static final Distance DEFAULT_BUMPER_HEIGHT = Inches.of(8);

  /** Center-to-center spacing of FUEL in the starting stacks, in meters. */
  private static final double STARTING_FUEL_SPACING = 0.152;

  private final Field2026 field;
  private final List<Fuel2026> fuelPieces = new ArrayList<>();
  private final List<Element> obstacles;
  private final List<Hub> hubs = new ArrayList<>();
  private final List<SimIntake> intakes = new ArrayList<>();
  private final Hub blueHub;
  private final Hub redHub;

  /** Broadphase grid over the FIELD footprint, holding indices into {@link #fuelPieces}. */
  private final List<List<Integer>> gridCells = new ArrayList<>();
  private final List<List<Integer>> activeCells = new ArrayList<>();
  private final int gridColumns;
  private final int gridRows;

  private final StructArrayPublisher<Pose3d> posePublisher;
  private final StructArrayPublisher<Pose3d> intakeZonePublisher;
  private final IntegerPublisher blueScorePublisher;
  private final IntegerPublisher redScorePublisher;

  private double bumperHeightMeters = DEFAULT_BUMPER_HEIGHT.in(Meters);
  private int subticks = DEFAULT_SUBTICKS;
  private boolean simulateAirResistance = false;
  private double loggingFrequencyHz = DEFAULT_LOGGING_FREQUENCY_HZ;

  /** Saturated so the very first {@link #process} call publishes immediately. */
  private double secondsSinceLastPublish = Double.MAX_VALUE;

  private Pose2d lastRobotPose;
  private Translation2d lastRobotDimensions;
  private ChassisSpeeds lastFieldRelativeSpeeds = new ChassisSpeeds();

  /**
   * Creates a FUEL simulation over {@code field}, pre-loaded with the match-start FUEL layout and
   * publishing to {@code Mechanisms/Fuel} on NetworkTables.
   *
   * @param field The season field this FUEL rolls around on.
   */
  public FuelLayer(Field2026 field) {
    this(field, "Fuel");
  }

  /**
   * Creates a FUEL simulation over {@code field}, pre-loaded with the match-start FUEL layout (see
   * {@link #spawnStartingFuel()}; call {@link #clearFuel()} for an empty FIELD instead).
   *
   * @param field The season field this FUEL rolls around on.
   * @param ntSubTableName Subtable of {@code Mechanisms} to publish ball poses and HUB scores to.
   */
  public FuelLayer(Field2026 field, String ntSubTableName) {
    this.field = field;
    this.obstacles = field.getGamePieceObstacles();

    this.blueHub = new Hub(
        field.getBlueHubCenter(), field.getBlueHubExit(), field.getBlueHubNet(),
        field.getHubEntryRadius(), field.getHubEntryHeight());
    this.redHub = new Hub(
        field.getRedHubCenter(), field.getRedHubExit(), field.getRedHubNet(),
        field.getHubEntryRadius(), field.getHubEntryHeight());
    hubs.add(blueHub);
    hubs.add(redHub);

    this.gridColumns = (int) Math.ceil(field.getFieldLength() / CELL_SIZE);
    this.gridRows = (int) Math.ceil(field.getFieldWidth() / CELL_SIZE);
    for (int i = 0; i < gridColumns * gridRows; i++) {
      gridCells.add(new ArrayList<>());
    }

    NetworkTable table =
        NetworkTableInstance.getDefault().getTable("Mechanisms").getSubTable(ntSubTableName);
    this.posePublisher = table.getStructArrayTopic("poses", Pose3d.struct).publish();
    this.intakeZonePublisher = table.getStructArrayTopic("intakeZones", Pose3d.struct).publish();
    this.blueScorePublisher = table.getIntegerTopic("blueScore").publish();
    this.redScorePublisher = table.getIntegerTopic("redScore").publish();

    spawnStartingFuel();
  }

  /**
   * Sets how high the top of the robot's bumpers sits above the carpet. FUEL below this height is
   * shoved aside by the robot driving into it; FUEL above it flies over the robot. Intakes decide
   * for themselves what they can reach, from the height of their own pickup box -- except for the
   * flat-footprint {@link #registerIntake(Transform2d, Transform2d) overload}, which spans exactly
   * this height and so must be given its footprint after this is set.
   *
   * @return This layer, for chaining.
   */
  public FuelLayer withBumperHeight(Distance bumperHeight) {
    this.bumperHeightMeters = bumperHeight.in(Meters);
    return this;
  }

  /**
   * Sets how many physics iterations run per {@link #process} call. More iterations cost more CPU
   * but keep fast-moving balls from tunnelling through thin structures.
   *
   * @param subticks Iterations per call, at least 1. Defaults to {@value #DEFAULT_SUBTICKS}.
   * @return This layer, for chaining.
   */
  public FuelLayer withSubticks(int subticks) {
    this.subticks = Math.max(1, subticks);
    return this;
  }

  /**
   * Enables or disables aerodynamic drag on airborne FUEL. Off by default, matching the simpler
   * ballistic model most shooter tuning starts from.
   *
   * @return This layer, for chaining.
   */
  public FuelLayer withAirResistance(boolean simulateAirResistance) {
    this.simulateAirResistance = simulateAirResistance;
    return this;
  }

  /**
   * Sets how often ball poses are published to NetworkTables. Publishing several hundred poses
   * every loop is what makes a FUEL sim feel slow in AdvantageScope, so this is throttled well
   * below the physics rate.
   *
   * @param loggingFrequencyHz Publish rate in Hz. Defaults to {@value #DEFAULT_LOGGING_FREQUENCY_HZ}.
   * @return This layer, for chaining.
   */
  public FuelLayer withLoggingFrequency(double loggingFrequencyHz) {
    this.loggingFrequencyHz = loggingFrequencyHz;
    return this;
  }

  /**
   * Steps the FUEL simulation, then hands {@code inputSpeeds} straight back: FUEL never
   * constrains the drivetrain (see the class javadoc), it only reacts to it.
   *
   * @param currentPose Current ground-truth field position of the robot.
   * @param inputSpeeds Robot-relative chassis speeds, used to shove FUEL the robot drives into.
   * @param robotDimensions Half-length (X) and half-width (Y) bumper dimensions in meters.
   * @param dtSeconds Time elapsed since the previous update step, in seconds.
   * @return {@code inputSpeeds}, unchanged.
   */
  @Override
  public ChassisSpeeds process(
      Pose2d currentPose, ChassisSpeeds inputSpeeds, Translation2d robotDimensions, double dtSeconds) {
    lastRobotPose = currentPose;
    lastRobotDimensions = robotDimensions;
    lastFieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(inputSpeeds, currentPose.getRotation());

    if (dtSeconds > 0) {
      stepSim(dtSeconds);

      secondsSinceLastPublish += dtSeconds;
      if (loggingFrequencyHz > 0 && secondsSinceLastPublish >= 1.0 / loggingFrequencyHz) {
        secondsSinceLastPublish = 0.0;
        publish();
      }
    }

    return inputSpeeds;
  }

  /**
   * Advances the FUEL simulation by {@code dtSeconds}, in {@link #withSubticks(int) subticks}
   * steps, against the most recently seen robot pose and speeds.
   *
   * <p>{@link #process} calls this for you; call it directly only to run the FUEL sim standalone,
   * outside a {@code SwerveDrivePhysics} pipeline.
   *
   * @param dtSeconds Time to advance, in seconds.
   */
  public void stepSim(double dtSeconds) {
    double subDt = dtSeconds / subticks;
    Translation2d robotVelocity = new Translation2d(
        lastFieldRelativeSpeeds.vxMetersPerSecond, lastFieldRelativeSpeeds.vyMetersPerSecond);

    for (int tick = 0; tick < subticks; tick++) {
      for (Fuel2026 fuel : fuelPieces) {
        integrate(fuel, subDt);
        handleFieldCollisions(fuel, subDt);
      }

      handleFuelCollisions();

      if (lastRobotPose != null && lastRobotDimensions != null) {
        for (Fuel2026 fuel : fuelPieces) {
          handleRobotCollision(fuel, lastRobotPose, lastRobotDimensions, robotVelocity);
        }
        handleIntakes(lastRobotPose);
      }
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Spawning
  // ---------------------------------------------------------------------------------------------

  /**
   * Fills the FIELD with the match-start FUEL layout: the two stacks straddling the FIELD's
   * center line, plus each alliance's DEPOT. Runs once from the constructor, so most callers only
   * need this to re-load the FIELD after a {@link #clearFuel()}.
   */
  public void spawnStartingFuel() {
    double halfSpacing = STARTING_FUEL_SPACING / 2.0;
    Translation3d center = new Translation3d(
        field.getFieldLength() / 2.0, field.getFieldWidth() / 2.0, FUEL_RADIUS);

    // Two mirrored 15x6 blocks either side of the center line, held apart by the center tape.
    for (int row = 0; row < 15; row++) {
      for (int column = 0; column < 6; column++) {
        double offsetX = halfSpacing + STARTING_FUEL_SPACING * column;
        double offsetY = 0.0254 + halfSpacing + STARTING_FUEL_SPACING * row;
        spawnFuel(center.plus(new Translation3d(offsetX, offsetY, 0)));
        spawnFuel(center.plus(new Translation3d(-offsetX, offsetY, 0)));
        spawnFuel(center.plus(new Translation3d(offsetX, -offsetY, 0)));
        spawnFuel(center.plus(new Translation3d(-offsetX, -offsetY, 0)));
      }
    }

    spawnDepotFuel(field.getBlueDepot(), true);
    spawnDepotFuel(field.getRedDepot(), false);
  }

  /**
   * Fills one DEPOT with a 3x4 block of FUEL either side of its center line, stacked away from the
   * ALLIANCE WALL the DEPOT sits against.
   *
   * @param depot The DEPOT element to fill.
   * @param againstMinX Whether the DEPOT's ALLIANCE WALL is at its low-X edge (the blue DEPOT).
   */
  private void spawnDepotFuel(Element depot, boolean againstMinX) {
    double halfSpacing = STARTING_FUEL_SPACING / 2.0;
    double centerY = depot.getCenter().getY();

    for (int row = 0; row < 3; row++) {
      for (int column = 0; column < 4; column++) {
        double offsetFromWall = halfSpacing + STARTING_FUEL_SPACING * column;
        double x = againstMinX
            ? depot.getMinX() + offsetFromWall
            : depot.getMaxX() - offsetFromWall;
        double offsetY = halfSpacing + STARTING_FUEL_SPACING * row;
        spawnFuel(new Translation3d(x, centerY + offsetY, FUEL_RADIUS));
        spawnFuel(new Translation3d(x, centerY - offsetY, FUEL_RADIUS));
      }
    }
  }

  /** Removes every FUEL ball from the FIELD. */
  public void clearFuel() {
    fuelPieces.clear();
  }

  /**
   * Drops a stationary FUEL ball onto the FIELD.
   *
   * @param position Field-relative position of the ball's center, in meters.
   * @return The spawned ball.
   */
  public Fuel2026 spawnFuel(Translation3d position) {
    return spawnFuel(position, new Translation3d());
  }

  /**
   * Adds a moving FUEL ball to the FIELD.
   *
   * @param position Field-relative position of the ball's center, in meters.
   * @param velocity Field-relative velocity, in meters per second.
   * @return The spawned ball.
   */
  public Fuel2026 spawnFuel(Translation3d position, Translation3d velocity) {
    Fuel2026 fuel = new Fuel2026(position, velocity);
    fuelPieces.add(fuel);
    return fuel;
  }

  /**
   * Shoots a FUEL ball out of the robot, accounting for the robot's own motion.
   *
   * @param launchVelocity Speed the ball leaves the shooter at.
   * @param hoodAngle Launch elevation, where zero is horizontal and 90 degrees is straight up.
   * @param turretYaw <i>Robot-relative</i> heading the ball is launched along.
   * @param launchHeight Height the ball leaves the robot at. Keep this above the bumper height, or
   *     the ball collides with the robot that just shot it.
   * @return The launched ball.
   * @throws IllegalStateException If the layer hasn't seen a robot pose yet, i.e. {@link #process}
   *     has never run.
   */
  public Fuel2026 launchFuel(
      LinearVelocity launchVelocity, Angle hoodAngle, Angle turretYaw, Distance launchHeight) {
    if (lastRobotPose == null) {
      throw new IllegalStateException(
          "Cannot launch FUEL before this layer has run a physics step -- add it to a "
              + "SwerveDrivePhysics pipeline, or call process() at least once.");
    }

    Pose3d launchPose = new Pose3d(lastRobotPose).plus(new Transform3d(
        new Translation3d(Meters.zero(), Meters.zero(), launchHeight), Rotation3d.kZero));

    double speed = launchVelocity.in(MetersPerSecond);
    double horizontalSpeed = Math.cos(hoodAngle.in(Radians)) * speed;
    double verticalSpeed = Math.sin(hoodAngle.in(Radians)) * speed;
    double fieldYaw = turretYaw.plus(launchPose.getRotation().getMeasureZ()).in(Radians);

    return spawnFuel(
        launchPose.getTranslation(),
        new Translation3d(
            horizontalSpeed * Math.cos(fieldYaw) + lastFieldRelativeSpeeds.vxMetersPerSecond,
            horizontalSpeed * Math.sin(fieldYaw) + lastFieldRelativeSpeeds.vyMetersPerSecond,
            verticalSpeed));
  }

  // ---------------------------------------------------------------------------------------------
  // Integration
  // ---------------------------------------------------------------------------------------------

  /**
   * Moves a ball by one substep and, unless it is resting on something, accelerates it under
   * gravity and drag.
   */
  private void integrate(Fuel2026 fuel, double dtSeconds) {
    fuel.translate(fuel.getVelocity().times(dtSeconds));

    if (!fuel.isSupported()) {
      Translation3d force = new Translation3d(0, 0, GRAVITY * FUEL_MASS_KG);
      if (simulateAirResistance) {
        double speed = fuel.getVelocity().getNorm();
        if (speed > 1e-6) {
          force = force.plus(fuel.getVelocity().times(-DRAG_FORCE_FACTOR * speed));
        }
      }
      fuel.addImpulse(force.div(FUEL_MASS_KG).times(dtSeconds));
    }

    // Cleared here so this substep's collisions decide afresh whether the ball is still supported.
    fuel.setSupported(false);
  }

  /**
   * Collides a ball against the carpet, every {@link Field2026#getGamePieceObstacles() field
   * structure}, the FIELD boundary and both HUBS, then lets it settle if it came to rest.
   */
  private void handleFieldCollisions(Fuel2026 fuel, double dtSeconds) {
    collideCarpet(fuel);

    for (Element obstacle : obstacles) {
      if (obstacle.isSloped()) {
        collideSlopedTop(fuel, obstacle);
      } else {
        collideBox(fuel, obstacle, FIELD_COR);
      }
    }

    clampToFieldBounds(fuel);

    for (Hub hub : hubs) {
      hub.handleFuelInteraction(fuel, dtSeconds);
    }

    settle(fuel, dtSeconds);
  }

  /**
   * Drops a ball that is touching the carpet back onto it, bouncing whatever downwards speed it
   * had left. A ball hovering within {@link #CONTACT_SKIN} of the carpet with almost no vertical
   * speed left settles onto it instead of bouncing.
   */
  private void collideCarpet(Fuel2026 fuel) {
    Translation3d position = fuel.getPosition();
    Translation3d velocity = fuel.getVelocity();
    boolean overlapping = position.getZ() < FUEL_RADIUS;
    if (!overlapping
        && (position.getZ() > FUEL_RADIUS + CONTACT_SKIN
            || Math.abs(velocity.getZ()) >= REST_SPEED)) {
      return;
    }

    fuel.setPosition(new Translation3d(position.getX(), position.getY(), FUEL_RADIUS));
    if (overlapping && velocity.getZ() < 0) {
      fuel.setVelocity(new Translation3d(
          velocity.getX(), velocity.getY(), -velocity.getZ() * FIELD_COR));
    }
    fuel.setSupported(true);
  }

  /**
   * Settles a ball that is touching a surface flat enough to hold it and has nearly stopped
   * moving vertically: its remaining vertical speed is dropped so it stops jittering, and its
   * horizontal speed bleeds off to rolling friction. A ball still moving vertically is left
   * unsupported so gravity keeps acting on it next substep.
   */
  private void settle(Fuel2026 fuel, double dtSeconds) {
    if (!fuel.isSupported()) {
      return;
    }
    if (Math.abs(fuel.getVelocity().getZ()) >= REST_SPEED) {
      fuel.setSupported(false);
      return;
    }

    Translation3d velocity = fuel.getVelocity();
    fuel.setVelocity(new Translation3d(velocity.getX(), velocity.getY(), 0.0)
        .times(1.0 - ROLLING_FRICTION * dtSeconds));
  }

  // ---------------------------------------------------------------------------------------------
  // Field structure collisions
  // ---------------------------------------------------------------------------------------------

  /**
   * Resolves a ball overlapping an upright field structure, treated as the axis-aligned box given
   * by the element's footprint and vertical extent.
   *
   * <p>The ball is pushed out along the direction from the nearest point of the box to its center,
   * so the same routine handles being shoved sideways off a HUB wall and coming to rest on top of
   * a TRENCH. A ball whose center has ended up inside the box escapes along whichever face it is
   * closest to.
   *
   * @param cor Coefficient of restitution to bounce the ball off this structure with.
   */
  private void collideBox(Fuel2026 fuel, Element element, double cor) {
    double minX = element.getMinX();
    double maxX = element.getMaxX();
    double minY = element.getMinY();
    double maxY = element.getMaxY();
    double minZ = element.getBottomHeight();
    double maxZ = element.getTopHeight();

    Translation3d position = fuel.getPosition();
    double x = position.getX();
    double y = position.getY();
    double z = position.getZ();

    double reach = FUEL_RADIUS + CONTACT_SKIN;
    if (x < minX - reach || x > maxX + reach
        || y < minY - reach || y > maxY + reach
        || z < minZ - reach || z > maxZ + reach) {
      return;
    }

    double deltaX = x - MathUtil.clamp(x, minX, maxX);
    double deltaY = y - MathUtil.clamp(y, minY, maxY);
    double deltaZ = z - MathUtil.clamp(z, minZ, maxZ);
    double distanceSquared = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
    if (distanceSquared > reach * reach) {
      return;
    }

    Translation3d normal;
    double depth;
    if (distanceSquared > 1e-12) {
      double distance = Math.sqrt(distanceSquared);
      normal = new Translation3d(deltaX / distance, deltaY / distance, deltaZ / distance);
      depth = FUEL_RADIUS - distance;
    } else {
      // Center is inside the box: leave through the closest face.
      double[] faceDistances = {x - minX, maxX - x, y - minY, maxY - y, z - minZ, maxZ - z};
      int closestFace = 0;
      for (int face = 1; face < faceDistances.length; face++) {
        if (faceDistances[face] < faceDistances[closestFace]) {
          closestFace = face;
        }
      }
      normal = switch (closestFace) {
        case 0 -> new Translation3d(-1, 0, 0);
        case 1 -> new Translation3d(1, 0, 0);
        case 2 -> new Translation3d(0, -1, 0);
        case 3 -> new Translation3d(0, 1, 0);
        case 4 -> new Translation3d(0, 0, -1);
        default -> new Translation3d(0, 0, 1);
      };
      depth = FUEL_RADIUS + faceDistances[closestFace];
    }

    if (depth < 0 && !isRestingContact(fuel, normal)) {
      return; // Inside the contact skin but not settling onto a top face: not touching yet.
    }

    fuel.translate(normal.times(depth));
    bounce(fuel, normal, cor);
  }

  /**
   * Whether a ball that is close to -- but not yet overlapping -- a surface should be treated as
   * resting on it: the surface has to be flat enough to hold the ball up, and the ball has to have
   * nearly stopped moving vertically. See {@link #CONTACT_SKIN}.
   */
  private boolean isRestingContact(Fuel2026 fuel, Translation3d normal) {
    return normal.getZ() > SUPPORT_NORMAL_Z && Math.abs(fuel.getVelocity().getZ()) < REST_SPEED;
  }

  /**
   * Resolves a ball rolling on a field structure whose top face slopes along X, such as a BUMP
   * face. The sloped face is treated as a line in the XZ plane spanning the element's footprint;
   * a ball within the element's Y band and closer to that line than its own radius gets pushed
   * out perpendicular to the slope, so it rolls up and over rather than stopping against a wall.
   */
  private void collideSlopedTop(Fuel2026 fuel, Element element) {
    Translation3d position = fuel.getPosition();
    if (position.getY() < element.getMinY() || position.getY() > element.getMaxY()) {
      return;
    }

    // Collapse to the XZ plane, where the sloped face is a single line segment.
    Translation2d start = new Translation2d(element.getMinX(), element.getTopHeightAtMinX());
    Translation2d end = new Translation2d(element.getMaxX(), element.getTopHeightAtMaxX());
    Translation2d ballXz = new Translation2d(position.getX(), position.getZ());
    Translation2d segment = end.minus(start);

    double segmentLength = segment.getNorm();
    if (segmentLength < 1e-9) {
      return;
    }

    Translation2d closest = start.plus(
        segment.times(ballXz.minus(start).dot(segment) / (segmentLength * segmentLength)));
    if (closest.getDistance(start) + closest.getDistance(end) > segmentLength) {
      return; // Nearest point on the infinite line falls outside the face itself.
    }

    double distance = ballXz.getDistance(closest);
    if (distance > FUEL_RADIUS + CONTACT_SKIN) {
      return;
    }

    Translation3d normal = new Translation3d(
        -segment.getY() / segmentLength, 0, segment.getX() / segmentLength);
    if (distance > FUEL_RADIUS && !isRestingContact(fuel, normal)) {
      return; // Inside the contact skin but still moving: not touching the face yet.
    }

    fuel.translate(normal.times(FUEL_RADIUS - distance));
    bounce(fuel, normal, FIELD_COR);
  }

  /**
   * Reflects a ball's velocity off a surface with the given outward {@code normal}, and marks the
   * ball supported if that surface is flat enough to rest on. A ball already moving away from the
   * surface keeps its velocity -- it has been pushed clear and shouldn't be pulled back.
   */
  private void bounce(Fuel2026 fuel, Translation3d normal, double cor) {
    double approachSpeed = fuel.getVelocity().dot(normal);
    if (approachSpeed < 0) {
      fuel.addImpulse(normal.times(-(1.0 + cor) * approachSpeed));
    }
    if (normal.getZ() > SUPPORT_NORMAL_Z) {
      fuel.setSupported(true);
    }
  }

  /**
   * Keeps a ball inside the FIELD perimeter at every height. The perimeter elements themselves
   * only reach as high as the netting above the guardrails, so this is the backstop that stops a
   * wild shot from leaving the simulation entirely.
   */
  private void clampToFieldBounds(Fuel2026 fuel) {
    Translation3d position = fuel.getPosition();
    Translation3d velocity = fuel.getVelocity();
    double maxX = field.getFieldLength() - FUEL_RADIUS;
    double maxY = field.getFieldWidth() - FUEL_RADIUS;

    if (position.getX() < FUEL_RADIUS && velocity.getX() < 0) {
      fuel.setPosition(new Translation3d(FUEL_RADIUS, position.getY(), position.getZ()));
      fuel.addImpulse(new Translation3d(-(1.0 + FIELD_COR) * velocity.getX(), 0, 0));
    } else if (position.getX() > maxX && velocity.getX() > 0) {
      fuel.setPosition(new Translation3d(maxX, position.getY(), position.getZ()));
      fuel.addImpulse(new Translation3d(-(1.0 + FIELD_COR) * velocity.getX(), 0, 0));
    }

    position = fuel.getPosition();
    velocity = fuel.getVelocity();
    if (position.getY() < FUEL_RADIUS && velocity.getY() < 0) {
      fuel.setPosition(new Translation3d(position.getX(), FUEL_RADIUS, position.getZ()));
      fuel.addImpulse(new Translation3d(0, -(1.0 + FIELD_COR) * velocity.getY(), 0));
    } else if (position.getY() > maxY && velocity.getY() > 0) {
      fuel.setPosition(new Translation3d(position.getX(), maxY, position.getZ()));
      fuel.addImpulse(new Translation3d(0, -(1.0 + FIELD_COR) * velocity.getY(), 0));
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Ball-to-ball collisions
  // ---------------------------------------------------------------------------------------------

  /**
   * Resolves every overlapping pair of balls. Pairs are found through a uniform grid over the
   * FIELD footprint, so a FIELD's worth of FUEL costs a scan of each ball's own cell and its eight
   * neighbours instead of comparing all several hundred against each other.
   */
  private void handleFuelCollisions() {
    for (List<Integer> cell : activeCells) {
      cell.clear();
    }
    activeCells.clear();

    for (int i = 0; i < fuelPieces.size(); i++) {
      int cellIndex = cellIndexOf(fuelPieces.get(i));
      if (cellIndex < 0) {
        continue;
      }
      List<Integer> cell = gridCells.get(cellIndex);
      cell.add(i);
      if (cell.size() == 1) {
        activeCells.add(cell);
      }
    }

    double diameter = FUEL_RADIUS * 2.0;
    for (int i = 0; i < fuelPieces.size(); i++) {
      Fuel2026 fuel = fuelPieces.get(i);
      int column = columnOf(fuel);
      int row = rowOf(fuel);

      for (int c = column - 1; c <= column + 1; c++) {
        for (int r = row - 1; r <= row + 1; r++) {
          if (c < 0 || c >= gridColumns || r < 0 || r >= gridRows) {
            continue;
          }
          for (int other : gridCells.get(c * gridRows + r)) {
            // Each pair is resolved once, by its lower-indexed ball.
            if (other <= i) {
              continue;
            }
            Fuel2026 otherFuel = fuelPieces.get(other);
            if (fuel.getPosition().getDistance(otherFuel.getPosition()) < diameter) {
              handleFuelCollision(fuel, otherFuel);
            }
          }
        }
      }
    }
  }

  /** Separates two overlapping balls and exchanges an equal and opposite impulse between them. */
  private static void handleFuelCollision(Fuel2026 a, Fuel2026 b) {
    Translation3d offset = a.getPosition().minus(b.getPosition());
    double distance = offset.getNorm();
    Translation3d normal = distance > 1e-9
        ? offset.div(distance)
        : new Translation3d(1, 0, 0); // Perfectly coincident: shove them apart arbitrarily.
    if (distance <= 1e-9) {
      distance = 0.0;
    }

    double overlap = FUEL_RADIUS * 2.0 - distance;
    a.translate(normal.times(overlap / 2.0));
    b.translate(normal.times(-overlap / 2.0));

    double impulse = 0.5 * (1.0 + FUEL_COR) * b.getVelocity().minus(a.getVelocity()).dot(normal);
    a.addImpulse(normal.times(impulse));
    b.addImpulse(normal.times(-impulse));
  }

  private int columnOf(Fuel2026 fuel) {
    return (int) Math.floor(fuel.getPosition().getX() / CELL_SIZE);
  }

  private int rowOf(Fuel2026 fuel) {
    return (int) Math.floor(fuel.getPosition().getY() / CELL_SIZE);
  }

  /** Flat index of the grid cell a ball sits in, or -1 if it is somehow off the FIELD. */
  private int cellIndexOf(Fuel2026 fuel) {
    int column = columnOf(fuel);
    int row = rowOf(fuel);
    if (column < 0 || column >= gridColumns || row < 0 || row >= gridRows) {
      return -1;
    }
    return column * gridRows + row;
  }

  // ---------------------------------------------------------------------------------------------
  // Robot interaction
  // ---------------------------------------------------------------------------------------------

  /**
   * Shoves a ball out of the robot's bumper footprint. The ball leaves along whichever bumper face
   * it is closest to, bouncing off it and picking up the robot's own speed into that face, so a
   * driving robot plows FUEL along in front of it.
   *
   * @param robotDimensions Half-length (X) and half-width (Y) bumper dimensions in meters.
   * @param robotVelocity Field-relative translational velocity of the robot, in m/s.
   */
  private void handleRobotCollision(
      Fuel2026 fuel, Pose2d robotPose, Translation2d robotDimensions, Translation2d robotVelocity) {
    if (fuel.getPosition().getZ() > bumperHeightMeters) {
      return; // Flying over the robot.
    }

    Translation2d relativePosition = new Pose2d(fuel.getTranslation2d(), Rotation2d.kZero)
        .relativeTo(robotPose)
        .getTranslation();

    // Distance the ball would have to travel to clear each bumper face; all four are negative
    // exactly when the ball is inside the footprint.
    double toBack = -FUEL_RADIUS - robotDimensions.getX() - relativePosition.getX();
    double toFront = -FUEL_RADIUS - robotDimensions.getX() + relativePosition.getX();
    double toRight = -FUEL_RADIUS - robotDimensions.getY() - relativePosition.getY();
    double toLeft = -FUEL_RADIUS - robotDimensions.getY() + relativePosition.getY();
    if (toBack > 0 || toFront > 0 || toRight > 0 || toLeft > 0) {
      return;
    }

    Translation2d pushOut;
    if (toBack >= toFront && toBack >= toRight && toBack >= toLeft) {
      pushOut = new Translation2d(toBack, 0);
    } else if (toFront >= toBack && toFront >= toRight && toFront >= toLeft) {
      pushOut = new Translation2d(-toFront, 0);
    } else if (toRight >= toBack && toRight >= toFront && toRight >= toLeft) {
      pushOut = new Translation2d(0, toRight);
    } else {
      pushOut = new Translation2d(0, -toLeft);
    }

    pushOut = pushOut.rotateBy(robotPose.getRotation());
    fuel.translate(new Translation3d(pushOut));

    Translation2d normal = pushOut.div(pushOut.getNorm());
    Translation2d horizontalVelocity = fuel.getVelocity().toTranslation2d();
    double approachSpeed = horizontalVelocity.dot(normal);
    if (approachSpeed < 0) {
      fuel.addImpulse(new Translation3d(normal.times(-approachSpeed * (1.0 + ROBOT_COR))));
    }

    double robotSpeedIntoBall = robotVelocity.dot(normal);
    if (robotSpeedIntoBall > 0) {
      fuel.addImpulse(new Translation3d(normal.times(robotSpeedIntoBall)));
    }
  }

  /**
   * Moves every intake's pickup box to follow the robot, then removes the FUEL sitting inside any
   * active one and fires its callback.
   */
  private void handleIntakes(Pose2d robotPose) {
    for (SimIntake intake : intakes) {
      intake.update(robotPose);
      if (!intake.isEnabled()) {
        continue;
      }
      for (int i = 0; i < fuelPieces.size(); i++) {
        Fuel2026 fuel = fuelPieces.get(i);
        if (!intake.contains(fuel.getPosition())) {
          continue;
        }

        fuel.setIntaked(true);
        fuelPieces.remove(i--);
        intake.onIntake.run();
      }
    }
  }

  /**
   * Registers an intake that removes FUEL from the FIELD whenever a ball enters its pickup box.
   *
   * <p>The two transforms are opposite corners of that box, measured from the robot's center on
   * the carpet -- so {@code x} is forwards, {@code y} is left, and {@code z} is up off the floor.
   * The box travels and rotates with the robot every physics step, so a ball is picked up only
   * where the mechanism actually is. Because the box states its own height, this is also what
   * decides whether a ball is low enough to be picked up: an over-the-bumper intake wants a
   * {@code z} range starting at its own lip, a ground intake one starting at zero.
   *
   * <p>Only the transforms' translations are used; the box is axis-aligned in the robot's frame.
   *
   * <pre>{@code
   * // A ground intake spanning the full bumper width, reaching 8in past the front bumper.
   * fuel.registerIntake(
   *     new Transform3d(Inches.of(14), Inches.of(-14), Inches.zero(), Rotation3d.kZero),
   *     new Transform3d(Inches.of(22), Inches.of(14), Inches.of(6), Rotation3d.kZero),
   *     intake::isRunning,
   *     intake::onFuelAcquired);
   * }</pre>
   *
   * @param cornerA One corner of the pickup box, relative to the robot's center.
   * @param cornerB The opposite corner of the pickup box, relative to the robot's center.
   * @param enabled Whether the intake is currently able to pick FUEL up.
   * @param onIntake Called once per ball picked up, e.g. to bump a held-piece count.
   * @return This layer, for chaining.
   */
  public FuelLayer registerIntake(
      Transform3d cornerA, Transform3d cornerB, BooleanSupplier enabled, Runnable onIntake) {
    intakes.add(new SimIntake(cornerA, cornerB, enabled, onIntake));
    return this;
  }

  /**
   * Registers an intake with no pickup callback.
   *
   * @see #registerIntake(Transform3d, Transform3d, BooleanSupplier, Runnable)
   */
  public FuelLayer registerIntake(
      Transform3d cornerA, Transform3d cornerB, BooleanSupplier enabled) {
    return registerIntake(cornerA, cornerB, enabled, () -> {});
  }

  /**
   * Registers an intake that is always running.
   *
   * @see #registerIntake(Transform3d, Transform3d, BooleanSupplier, Runnable)
   */
  public FuelLayer registerIntake(Transform3d cornerA, Transform3d cornerB) {
    return registerIntake(cornerA, cornerB, () -> true, () -> {});
  }

  /**
   * Registers an intake from a flat footprint, for the common ground intake that picks up anything
   * it drives over. The pickup box spans from the carpet up to the robot's
   * {@link #withBumperHeight(Distance) bumper height}, i.e. exactly the FUEL the robot could be
   * pushing around.
   *
   * @param cornerA One corner of the pickup footprint, relative to the robot's center.
   * @param cornerB The opposite corner of the pickup footprint, relative to the robot's center.
   * @param enabled Whether the intake is currently able to pick FUEL up.
   * @param onIntake Called once per ball picked up, e.g. to bump a held-piece count.
   * @return This layer, for chaining.
   * @see #registerIntake(Transform3d, Transform3d, BooleanSupplier, Runnable)
   */
  public FuelLayer registerIntake(
      Transform2d cornerA, Transform2d cornerB, BooleanSupplier enabled, Runnable onIntake) {
    return registerIntake(
        new Transform3d(
            cornerA.getX(), cornerA.getY(), 0.0, Rotation3d.kZero),
        new Transform3d(
            cornerB.getX(), cornerB.getY(), bumperHeightMeters, Rotation3d.kZero),
        enabled,
        onIntake);
  }

  /**
   * Registers a flat-footprint intake with no pickup callback.
   *
   * @see #registerIntake(Transform2d, Transform2d, BooleanSupplier, Runnable)
   */
  public FuelLayer registerIntake(
      Transform2d cornerA, Transform2d cornerB, BooleanSupplier enabled) {
    return registerIntake(cornerA, cornerB, enabled, () -> {});
  }

  /**
   * Registers a flat-footprint intake that is always running.
   *
   * @see #registerIntake(Transform2d, Transform2d, BooleanSupplier, Runnable)
   */
  public FuelLayer registerIntake(Transform2d cornerA, Transform2d cornerB) {
    return registerIntake(cornerA, cornerB, () -> true, () -> {});
  }

  /** Every registered intake, whose pickup boxes track the robot as the simulation steps. */
  public List<SimIntake> getIntakes() {
    return intakes;
  }

  // ---------------------------------------------------------------------------------------------
  // State access and publishing
  // ---------------------------------------------------------------------------------------------

  /** Every FUEL ball currently on the FIELD, for intake checks or custom visualization. */
  public List<Fuel2026> getFuelPieces() {
    return fuelPieces;
  }

  /** Field-relative center of every FUEL ball currently on the FIELD, in meters. */
  public List<Translation3d> getFuelTranslations() {
    List<Translation3d> translations = new ArrayList<>(fuelPieces.size());
    for (Fuel2026 fuel : fuelPieces) {
      translations.add(fuel.getPosition());
    }
    return translations;
  }

  /** Every FUEL ball's position as an (unrotated) 3D pose. */
  public List<Pose3d> getPose3dList() {
    List<Pose3d> poses = new ArrayList<>(fuelPieces.size());
    for (Fuel2026 fuel : fuelPieces) {
      poses.add(fuel.getPose3d());
    }
    return poses;
  }

  /** Every FUEL ball's position as an (unrotated) 3D pose, for struct array consumers. */
  public Pose3d[] getPose3dArray() {
    Pose3d[] poses = new Pose3d[fuelPieces.size()];
    for (int i = 0; i < poses.length; i++) {
      poses[i] = fuelPieces.get(i).getPose3d();
    }
    return poses;
  }

  /** The blue alliance's HUB, for reading and resetting its score. */
  public Hub getBlueHub() {
    return blueHub;
  }

  /** The red alliance's HUB, for reading and resetting its score. */
  public Hub getRedHub() {
    return redHub;
  }

  /**
   * Publishes the current ball poses, intake pickup boxes and HUB scores to NetworkTables
   * immediately, regardless of the {@link #withLoggingFrequency(double) publish rate}.
   */
  public void publish() {
    posePublisher.set(getPose3dArray());

    Pose3d[] intakeZones = new Pose3d[intakes.size()];
    for (int i = 0; i < intakeZones.length; i++) {
      intakeZones[i] = intakes.get(i).getCenterPose();
    }
    intakeZonePublisher.set(intakeZones);

    blueScorePublisher.set(blueHub.getScore());
    redScorePublisher.set(redHub.getScore());
  }

  /**
   * One alliance's HUB, wrapping the {@link Field2026} geometry that decides whether a ball has
   * been SCORED and where it goes afterwards.
   *
   * <p>A HUB's solid walls are collided with as an ordinary field structure, through
   * {@link Field2026#getGamePieceObstacles()}. What's left here is the part that isn't just
   * geometry: catching a ball dropping through the goal, counting it, dispersing it back onto the
   * FIELD, and the softer bounce off the backing net.
   */
  public static class Hub {

    private final Translation2d center;
    private final Translation3d exit;
    private final Element net;
    private final double entryRadius;
    private final double entryHeight;

    /** +1 if this HUB disperses scored FUEL towards +X, -1 towards -X. */
    private final double dispersalDirection;

    private int score = 0;

    private Hub(
        Translation2d center,
        Translation3d exit,
        Element net,
        double entryRadius,
        double entryHeight) {
      this.center = center;
      this.exit = exit;
      this.net = net;
      this.entryRadius = entryRadius;
      this.entryHeight = entryHeight;
      this.dispersalDirection = Math.signum(exit.getX() - center.getX());
    }

    /** Number of FUEL balls SCORED in this HUB since the last {@link #resetScore()}. */
    public int getScore() {
      return score;
    }

    /** Resets this HUB's score to zero, e.g. at the start of a simulated match. */
    public void resetScore() {
      score = 0;
    }

    /** Center of this HUB, in field-relative meters. */
    public Translation2d getCenter() {
      return center;
    }

    /** Scores a ball that just dropped through the goal, otherwise bounces it off the net. */
    private void handleFuelInteraction(Fuel2026 fuel, double dtSeconds) {
      if (didFuelScore(fuel, dtSeconds)) {
        score++;
        fuel.setPosition(exit);
        fuel.setVelocity(dispersalVelocity());
        fuel.setSupported(false);
        return;
      }
      collideNet(fuel);
    }

    /**
     * Whether a ball crossed the goal opening downwards during this substep -- inside the goal
     * radius, now below the opening, and above it before the substep moved it.
     */
    private boolean didFuelScore(Fuel2026 fuel, double dtSeconds) {
      Translation3d position = fuel.getPosition();
      return position.toTranslation2d().getDistance(center) <= entryRadius
          && position.getZ() <= entryHeight
          && position.minus(fuel.getVelocity().times(dtSeconds)).getZ() > entryHeight;
    }

    /** A shove back onto the FIELD away from the HUB, with some spread so returns fan out. */
    private Translation3d dispersalVelocity() {
      return new Translation3d(
          dispersalDirection * (Math.random() + 0.1) * 1.5, Math.random() * 2.0 - 1.0, 0);
    }

    /**
     * Bounces a ball off this HUB's backing net. The net is modeled as a single hanging plane
     * rather than a solid: a ball that reaches it is pushed out to whichever side of the plane its
     * center is already on, and keeps only a fraction of its speed, since netting absorbs an
     * impact instead of returning it.
     */
    private void collideNet(Fuel2026 fuel) {
      Translation3d position = fuel.getPosition();
      if (position.getZ() > net.getTopHeight() || position.getZ() < net.getBottomHeight()) {
        return;
      }
      if (position.getY() > net.getMaxY() || position.getY() < net.getMinY()) {
        return;
      }

      double planeX = net.getCenter().getX();
      double pushOutX = position.getX() > planeX
          ? Math.max(0.0, planeX - (position.getX() - FUEL_RADIUS))
          : Math.min(0.0, planeX - (position.getX() + FUEL_RADIUS));
      if (pushOutX == 0.0) {
        return;
      }

      fuel.translate(new Translation3d(pushOutX, 0, 0));
      Translation3d velocity = fuel.getVelocity();
      fuel.setVelocity(new Translation3d(
          -velocity.getX() * NET_COR, velocity.getY() * NET_COR, velocity.getZ()));
    }
  }

  /**
   * A 3D pickup box, fixed to the robot, that removes FUEL from the FIELD while it is enabled.
   *
   * <p>The box is stored as two opposite corners in the robot's own frame, so it is defined once
   * and then carried around by the robot: {@link #update(Pose2d)} re-places it from the robot's
   * latest pose every physics step, and {@link #contains(Translation3d)} tests a ball against it
   * where the robot actually is right now. {@link #getCenterPose()} and {@link #getSize()} hand
   * that same field-relative box back for visualization.
   */
  public static final class SimIntake {

    /** Corner of the box nearest the robot's origin on all three axes, robot-relative. */
    private final Translation3d minCorner;

    /** Corner of the box furthest from the robot's origin on all three axes, robot-relative. */
    private final Translation3d maxCorner;

    private final BooleanSupplier enabled;
    private final Runnable onIntake;

    /** Latest robot pose the box has been placed from. */
    private Pose3d robotPose = new Pose3d();

    /** Field-relative center of the box, re-derived by {@link #update(Pose2d)}. */
    private Pose3d centerPose;

    private SimIntake(
        Transform3d cornerA, Transform3d cornerB, BooleanSupplier enabled, Runnable onIntake) {
      Translation3d a = cornerA.getTranslation();
      Translation3d b = cornerB.getTranslation();
      this.minCorner = new Translation3d(
          Math.min(a.getX(), b.getX()), Math.min(a.getY(), b.getY()), Math.min(a.getZ(), b.getZ()));
      this.maxCorner = new Translation3d(
          Math.max(a.getX(), b.getX()), Math.max(a.getY(), b.getY()), Math.max(a.getZ(), b.getZ()));
      this.enabled = enabled;
      this.onIntake = onIntake;
      update(Pose2d.kZero);
    }

    /** Re-places this pickup box from the robot's current pose. */
    private void update(Pose2d newRobotPose) {
      this.robotPose = new Pose3d(newRobotPose);
      this.centerPose = robotPose.plus(
          new Transform3d(minCorner.plus(maxCorner).div(2.0), Rotation3d.kZero));
    }

    /** Whether this intake is currently able to pick FUEL up. */
    public boolean isEnabled() {
      return enabled.getAsBoolean();
    }

    /**
     * Whether {@code fieldPosition} is inside this pickup box as it currently sits on the FIELD.
     *
     * @param fieldPosition Field-relative position to test, in meters.
     */
    public boolean contains(Translation3d fieldPosition) {
      Translation3d relative =
          new Pose3d(fieldPosition, Rotation3d.kZero).relativeTo(robotPose).getTranslation();
      return relative.getX() >= minCorner.getX()
          && relative.getX() <= maxCorner.getX()
          && relative.getY() >= minCorner.getY()
          && relative.getY() <= maxCorner.getY()
          && relative.getZ() >= minCorner.getZ()
          && relative.getZ() <= maxCorner.getZ();
    }

    /**
     * Field-relative center of this pickup box, oriented with the robot -- as of the last physics
     * step. Publish this alongside {@link #getSize()} to draw the box in a 3D field view.
     */
    public Pose3d getCenterPose() {
      return centerPose;
    }

    /** Full size of this pickup box along the robot's X, Y and Z axes, in meters. */
    public Translation3d getSize() {
      return maxCorner.minus(minCorner);
    }

    /** Corner of this pickup box nearest the robot's origin on all three axes, robot-relative. */
    public Translation3d getMinCorner() {
      return minCorner;
    }

    /** Corner of this pickup box furthest from the robot's origin on all three axes, robot-relative. */
    public Translation3d getMaxCorner() {
      return maxCorner;
    }
  }
}
