package jsim.physics.layers;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
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
import jsim.physics.layers.fields.FieldLayout.Element;
import jsim.physics.layers.fields.Field2026;
import jsim.physics.layers.gamepieces.Fuel2026;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import jsim.physics.layers.utils.Contact;
import jsim.physics.layers.utils.Cuboid3d;

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

  /** Downwards acceleration due to gravity, in m/s^2. */
  private static final double GRAVITY = -9.81;

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

  /**
   * The FIELD's playable footprint, inset by one ball radius on every side, so a ball's own edge
   * (not its center) is what the FIELD boundary actually stops. Passed to
   * {@link Fuel2026#clampToBounds} every substep.
   */
  private final Rectangle2d fieldBounds;

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
    this.fieldBounds = new Rectangle2d(
        new Pose2d(field.getFieldLength() / 2.0, field.getFieldWidth() / 2.0, Rotation2d.kZero),
        field.getFieldLength() - 2.0 * Fuel2026.FUEL_RADIUS,
        field.getFieldWidth() - 2.0 * Fuel2026.FUEL_RADIUS);

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
   * @param bumperHeight Height of the top of the robot's bumpers above the carpet.
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
   * @param simulateAirResistance Whether airborne FUEL should feel aerodynamic drag.
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
   * <p>{@link #process} calls this for you.
   *
   * @param dtSeconds Time to advance, in seconds.
   */
  private void stepSim(double dtSeconds) {
    double subDt = dtSeconds / subticks;
    Translation2d robotVelocity = new Translation2d(
        lastFieldRelativeSpeeds.vxMetersPerSecond, lastFieldRelativeSpeeds.vyMetersPerSecond);

    for (int tick = 0; tick < subticks; tick++) {
      for (Fuel2026 fuel : fuelPieces) {
        fuel.integrate(subDt, GRAVITY, simulateAirResistance);
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
        field.getFieldLength() / 2.0, field.getFieldWidth() / 2.0, Fuel2026.FUEL_RADIUS);

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
        spawnFuel(new Translation3d(x, centerY + offsetY, Fuel2026.FUEL_RADIUS));
        spawnFuel(new Translation3d(x, centerY - offsetY, Fuel2026.FUEL_RADIUS));
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

  /**
   * Collides a ball against the carpet, every {@link Field2026#getGamePieceObstacles() field
   * structure}, the FIELD boundary and both HUBS, then lets it settle if it came to rest.
   *
   * @param fuel The ball to collide.
   * @param dtSeconds Substep duration, in seconds.
   */
  private void handleFieldCollisions(Fuel2026 fuel, double dtSeconds) {
    fuel.collideCarpet(CONTACT_SKIN, REST_SPEED);

    for (Element obstacle : obstacles) {
      if (obstacle.isSloped()) {
        fuel.collideSlopedTop(obstacle, CONTACT_SKIN, SUPPORT_NORMAL_Z, REST_SPEED);
      } else {
        fuel.collideBox(obstacle.getCuboid(), Fuel2026.FIELD_COR, CONTACT_SKIN, SUPPORT_NORMAL_Z, REST_SPEED);
      }
    }

    fuel.clampToBounds(fieldBounds, SUPPORT_NORMAL_Z);

    for (Hub hub : hubs) {
      hub.handleFuelInteraction(fuel, dtSeconds);
    }

    fuel.settle(dtSeconds, REST_SPEED);
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
      int cellIndex = fuelPieces.get(i).cellIndexOf(CELL_SIZE, gridColumns, gridRows);
      if (cellIndex < 0) {
        continue;
      }
      List<Integer> cell = gridCells.get(cellIndex);
      cell.add(i);
      if (cell.size() == 1) {
        activeCells.add(cell);
      }
    }

    double diameter = Fuel2026.FUEL_RADIUS * 2.0;
    for (int i = 0; i < fuelPieces.size(); i++) {
      Fuel2026 fuel = fuelPieces.get(i);
      int column = fuel.columnOf(CELL_SIZE);
      int row = fuel.rowOf(CELL_SIZE);

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
              fuel.collide(otherFuel, Fuel2026.FUEL_COR);
            }
          }
        }
      }
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Robot interaction
  // ---------------------------------------------------------------------------------------------

  /**
   * Shoves a ball out of the robot's bumper footprint. The ball leaves along whichever bumper face
   * it is closest to, bouncing off it and picking up the robot's own speed into that face, so a
   * driving robot plows FUEL along in front of it.
   *
   * @param fuel The ball to collide against the robot.
   * @param robotPose Current ground-truth field pose of the robot.
   * @param robotDimensions Half-length (X) and half-width (Y) bumper dimensions in meters.
   * @param robotVelocity Field-relative translational velocity of the robot, in m/s.
   */
  private void handleRobotCollision(
      Fuel2026 fuel, Pose2d robotPose, Translation2d robotDimensions, Translation2d robotVelocity) {
    Cuboid3d bumperBox = new Cuboid3d(
        new Pose3d(
            new Translation3d(robotPose.getX(), robotPose.getY(), bumperHeightMeters / 2.0),
            new Rotation3d(robotPose.getRotation())),
        robotDimensions.getX() * 2.0,
        robotDimensions.getY() * 2.0,
        bumperHeightMeters);

    Contact contact = fuel.collideBox(bumperBox, Fuel2026.ROBOT_COR, 0.0, SUPPORT_NORMAL_Z, REST_SPEED);
    if (contact == null) {
      return;
    }

    Translation3d robotVelocity3d = new Translation3d(robotVelocity.getX(), robotVelocity.getY(), 0);
    double robotSpeedIntoBall = robotVelocity3d.dot(contact.normal());
    if (robotSpeedIntoBall > 0) {
      fuel.addImpulse(contact.normal().times(robotSpeedIntoBall));
    }
  }

  /**
   * Moves every intake's pickup box to follow the robot, then removes the FUEL sitting inside any
   * active one and fires its callback.
   *
   * @param robotPose Current ground-truth field pose of the robot.
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
   * <p>{@code localBox} is defined once, in the robot's own frame -- so {@code x} is forwards,
   * {@code y} is left, and {@code z} is up off the floor -- and is then carried around by the
   * robot: {@link SimIntake#update} re-places it onto the FIELD from the robot's latest pose every
   * physics step, so a ball is picked up only where the mechanism actually is. Because the box
   * states its own height, this is also what decides whether a ball is low enough to be picked up:
   * an over-the-bumper intake wants a {@code z} range starting at its own lip, a ground intake one
   * starting at zero.
   *
   * <p>This is the primary {@code registerIntake} overload; the {@link Transform3d} and
   * {@link Transform2d}-corner overloads all build a {@link Cuboid3d} and delegate here.
   *
   * <pre>{@code
   * // A ground intake spanning the full bumper width, reaching 8in past the front bumper.
   * fuel.registerIntake(
   *     new Cuboid3d(
   *         new Translation3d(Inches.of(14), Inches.of(-14), Inches.zero()),
   *         new Translation3d(Inches.of(22), Inches.of(14), Inches.of(6))),
   *     intake::isRunning,
   *     intake::onFuelAcquired);
   * }</pre>
   *
   * @param localBox The pickup box, relative to the robot's center.
   * @param enabled Whether the intake is currently able to pick FUEL up.
   * @param onIntake Called once per ball picked up, e.g. to bump a held-piece count.
   * @return This layer, for chaining.
   */
  public FuelLayer registerIntake(Cuboid3d localBox, BooleanSupplier enabled, Runnable onIntake) {
    intakes.add(new SimIntake(localBox, enabled, onIntake));
    return this;
  }

  /**
   * Registers an intake with no pickup callback.
   *
   * @param localBox The pickup box, relative to the robot's center.
   * @param enabled Whether the intake is currently able to pick FUEL up.
   * @return This layer, for chaining.
   * @see #registerIntake(Cuboid3d, BooleanSupplier, Runnable)
   */
  public FuelLayer registerIntake(Cuboid3d localBox, BooleanSupplier enabled) {
    return registerIntake(localBox, enabled, () -> {});
  }

  /**
   * Registers an intake that is always running.
   *
   * @param localBox The pickup box, relative to the robot's center.
   * @return This layer, for chaining.
   * @see #registerIntake(Cuboid3d, BooleanSupplier, Runnable)
   */
  public FuelLayer registerIntake(Cuboid3d localBox) {
    return registerIntake(localBox, () -> true, () -> {});
  }

  /**
   * Registers an intake from two opposite corners of an axis-aligned pickup box, relative to the
   * robot's center. Only the transforms' translations are used.
   *
   * @param cornerA One corner of the pickup box, relative to the robot's center.
   * @param cornerB The opposite corner of the pickup box, relative to the robot's center.
   * @param enabled Whether the intake is currently able to pick FUEL up.
   * @param onIntake Called once per ball picked up, e.g. to bump a held-piece count.
   * @return This layer, for chaining.
   * @see #registerIntake(Cuboid3d, BooleanSupplier, Runnable)
   */
  public FuelLayer registerIntake(
      Transform3d cornerA, Transform3d cornerB, BooleanSupplier enabled, Runnable onIntake) {
    return registerIntake(
        new Cuboid3d(cornerA.getTranslation(), cornerB.getTranslation()), enabled, onIntake);
  }

  /**
   * Registers an intake with no pickup callback.
   *
   * @param cornerA One corner of the pickup box, relative to the robot's center.
   * @param cornerB The opposite corner of the pickup box, relative to the robot's center.
   * @param enabled Whether the intake is currently able to pick FUEL up.
   * @return This layer, for chaining.
   * @see #registerIntake(Transform3d, Transform3d, BooleanSupplier, Runnable)
   */
  public FuelLayer registerIntake(
      Transform3d cornerA, Transform3d cornerB, BooleanSupplier enabled) {
    return registerIntake(cornerA, cornerB, enabled, () -> {});
  }

  /**
   * Registers an intake that is always running.
   *
   * @param cornerA One corner of the pickup box, relative to the robot's center.
   * @param cornerB The opposite corner of the pickup box, relative to the robot's center.
   * @return This layer, for chaining.
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
   * @see #registerIntake(Cuboid3d, BooleanSupplier, Runnable)
   */
  public FuelLayer registerIntake(
      Transform2d cornerA, Transform2d cornerB, BooleanSupplier enabled, Runnable onIntake) {
    return registerIntake(
        new Cuboid3d(
            new Translation3d(cornerA.getX(), cornerA.getY(), 0.0),
            new Translation3d(cornerB.getX(), cornerB.getY(), bumperHeightMeters)),
        enabled,
        onIntake);
  }

  /**
   * Registers a flat-footprint intake with no pickup callback.
   *
   * @param cornerA One corner of the pickup footprint, relative to the robot's center.
   * @param cornerB The opposite corner of the pickup footprint, relative to the robot's center.
   * @param enabled Whether the intake is currently able to pick FUEL up.
   * @return This layer, for chaining.
   * @see #registerIntake(Transform2d, Transform2d, BooleanSupplier, Runnable)
   */
  public FuelLayer registerIntake(
      Transform2d cornerA, Transform2d cornerB, BooleanSupplier enabled) {
    return registerIntake(cornerA, cornerB, enabled, () -> {});
  }

  /**
   * Registers a flat-footprint intake that is always running.
   *
   * @param cornerA One corner of the pickup footprint, relative to the robot's center.
   * @param cornerB The opposite corner of the pickup footprint, relative to the robot's center.
   * @return This layer, for chaining.
   * @see #registerIntake(Transform2d, Transform2d, BooleanSupplier, Runnable)
   */
  public FuelLayer registerIntake(Transform2d cornerA, Transform2d cornerB) {
    return registerIntake(cornerA, cornerB, () -> true, () -> {});
  }

  /**
   * Returns every registered intake, whose pickup boxes track the robot as the simulation steps.
   *
   * @return Every registered intake.
   */
  public List<SimIntake> getIntakes() {
    return intakes;
  }

  // ---------------------------------------------------------------------------------------------
  // State access and publishing
  // ---------------------------------------------------------------------------------------------

  /**
   * Returns every FUEL ball currently on the FIELD, for intake checks or custom visualization.
   *
   * @return Every FUEL ball currently on the FIELD.
   */
  public List<Fuel2026> getFuelPieces() {
    return fuelPieces;
  }

  /**
   * Returns the field-relative center of every FUEL ball currently on the FIELD.
   *
   * @return Field-relative center of every FUEL ball currently on the FIELD, in meters.
   */
  public List<Translation3d> getFuelTranslations() {
    List<Translation3d> translations = new ArrayList<>(fuelPieces.size());
    for (Fuel2026 fuel : fuelPieces) {
      translations.add(fuel.getPosition());
    }
    return translations;
  }

  /**
   * Returns every FUEL ball's position as an (unrotated) 3D pose.
   *
   * @return Every FUEL ball's position as an unrotated {@link Pose3d}.
   */
  public List<Pose3d> getPose3dList() {
    List<Pose3d> poses = new ArrayList<>(fuelPieces.size());
    for (Fuel2026 fuel : fuelPieces) {
      poses.add(fuel.getPose3d());
    }
    return poses;
  }

  /**
   * Returns every FUEL ball's position as an (unrotated) 3D pose, for struct array consumers.
   *
   * @return Every FUEL ball's position as an unrotated {@link Pose3d} array.
   */
  public Pose3d[] getPose3dArray() {
    Pose3d[] poses = new Pose3d[fuelPieces.size()];
    for (int i = 0; i < poses.length; i++) {
      poses[i] = fuelPieces.get(i).getPose3d();
    }
    return poses;
  }

  /**
   * Returns the blue alliance's HUB, for reading and resetting its score.
   *
   * @return The blue alliance's {@link Hub}.
   */
  public Hub getBlueHub() {
    return blueHub;
  }

  /**
   * Returns the red alliance's HUB, for reading and resetting its score.
   *
   * @return The red alliance's {@link Hub}.
   */
  public Hub getRedHub() {
    return redHub;
  }

  /**
   * Publishes the current ball poses, intake pickup boxes and HUB scores to NetworkTables
   * immediately, regardless of the {@link #withLoggingFrequency(double) publish rate}.
   */
  private void publish() {
    posePublisher.set(getPose3dArray());

    Pose3d[] intakeZones = new Pose3d[intakes.size()];
    for (int i = 0; i < intakeZones.length; i++) {
      intakeZones[i] = intakes.get(i).getBox().getCenter();
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

    /**
     * Creates a HUB.
     *
     * @param center Center of this HUB, in field-relative meters.
     * @param exit Where scored FUEL re-enters play from this HUB.
     * @param net This HUB's backing net element.
     * @param entryRadius Radius of this HUB's open goal about its center, in meters.
     * @param entryHeight Height of this HUB's open goal above the carpet, in meters.
     */
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

    /**
     * Returns the number of FUEL balls SCORED in this HUB since the last {@link #resetScore()}.
     *
     * @return Number of FUEL balls SCORED in this HUB since the last reset.
     */
    public int getScore() {
      return score;
    }

    /** Resets this HUB's score to zero, e.g. at the start of a simulated match. */
    public void resetScore() {
      score = 0;
    }

    /**
     * Returns the center of this HUB.
     *
     * @return Center of this HUB, in field-relative meters.
     */
    public Translation2d getCenter() {
      return center;
    }

    /**
     * Scores a ball that just dropped through the goal, otherwise bounces it off the net.
     *
     * @param fuel The ball to check and resolve.
     * @param dtSeconds Substep duration, in seconds.
     */
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
     *
     * @param fuel The ball to check.
     * @param dtSeconds Substep duration, in seconds.
     * @return Whether {@code fuel} just scored in this HUB.
     */
    private boolean didFuelScore(Fuel2026 fuel, double dtSeconds) {
      Translation3d position = fuel.getPosition();
      return position.toTranslation2d().getDistance(center) <= entryRadius
          && position.getZ() <= entryHeight
          && position.minus(fuel.getVelocity().times(dtSeconds)).getZ() > entryHeight;
    }

    /**
     * A shove back onto the FIELD away from the HUB, with some spread so returns fan out.
     *
     * @return A randomized dispersal velocity, in meters per second.
     */
    private Translation3d dispersalVelocity() {
      return new Translation3d(
          dispersalDirection * (Math.random() + 0.1) * 1.5, Math.random() * 2.0 - 1.0, 0);
    }

    /**
     * Bounces a ball off this HUB's backing net. The net is modeled as a single hanging plane
     * rather than a solid: a ball that reaches it is pushed out to whichever side of the plane its
     * center is already on, and keeps only a fraction of its speed, since netting absorbs an
     * impact instead of returning it.
     *
     * @param fuel The ball to bounce off the net.
     */
    private void collideNet(Fuel2026 fuel) {
      fuel.collideBox(net.getCuboid(), Fuel2026.NET_COR, 0.0, SUPPORT_NORMAL_Z, REST_SPEED);
    }
  }

  /**
   * A 3D pickup box, fixed to the robot, that removes FUEL from the FIELD while it is enabled.
   *
   * <p>The box is stored as a {@link Cuboid3d} in the robot's own frame, so it is defined once and
   * then carried around by the robot: {@link SimIntake#update(Pose2d)} re-places it from the
   * robot's latest pose every physics step, and {@link SimIntake#contains(Translation3d)} tests a
   * ball against it where the robot actually is right now. {@link SimIntake#getBox()} hands that
   * same field-relative box back for visualization.
   */
  public static final class SimIntake {

    /** This pickup box, fixed in the robot's own frame. */
    private final Cuboid3d localBox;

    private final BooleanSupplier enabled;
    private final Runnable onIntake;

    /** This pickup box, re-placed onto the FIELD by {@link #update(Pose2d)} every physics step. */
    private Cuboid3d fieldBox;

    /**
     * Creates an intake.
     *
     * @param localBox The pickup box, in the robot's own frame.
     * @param enabled Whether the intake is currently able to pick FUEL up.
     * @param onIntake Called once per ball picked up.
     */
    private SimIntake(Cuboid3d localBox, BooleanSupplier enabled, Runnable onIntake) {
      this.localBox = localBox;
      this.enabled = enabled;
      this.onIntake = onIntake;
      update(Pose2d.kZero);
    }

    /**
     * Re-places this pickup box onto the FIELD from the robot's current pose, by reparenting
     * {@link #localBox}'s center -- an offset in the robot's frame -- into the field frame through
     * the robot's own field pose.
     *
     * @param robotPose Current ground-truth field pose of the robot.
     */
    private void update(Pose2d robotPose) {
      Pose3d robotPose3d = new Pose3d(robotPose);
      Pose3d fieldCenter = robotPose3d.plus(
          new Transform3d(localBox.getCenter().getTranslation(), localBox.getCenter().getRotation()));
      this.fieldBox = new Cuboid3d(
          fieldCenter, localBox.getXWidth(), localBox.getYWidth(), localBox.getZWidth());
    }

    /**
     * Returns whether this intake is currently able to pick FUEL up.
     *
     * @return Whether this intake is currently able to pick FUEL up.
     */
    public boolean isEnabled() {
      return enabled.getAsBoolean();
    }

    /**
     * Whether {@code fieldPosition} is inside this pickup box as it currently sits on the FIELD.
     *
     * @param fieldPosition Field-relative position to test, in meters.
     * @return Whether {@code fieldPosition} is inside this pickup box.
     */
    public boolean contains(Translation3d fieldPosition) {
      return fieldBox.contains(fieldPosition);
    }

    /**
     * Returns this pickup box's current field-relative placement, oriented with the robot -- as of
     * the last physics step. Publish this to draw the box in a 3D field view.
     *
     * @return This pickup box's current field-relative placement.
     */
    public Cuboid3d getBox() {
      return fieldBox;
    }
  }
}
