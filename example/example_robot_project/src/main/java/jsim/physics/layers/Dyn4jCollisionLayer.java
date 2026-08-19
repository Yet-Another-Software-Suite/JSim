package jsim.physics.layers;

import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Mass;
import jsim.physics.PhysicsLayer;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.World;

public class Dyn4jCollisionLayer implements PhysicsLayer {

  private final World<Body> world;
  private final Body robotBody;
  private final Mass mass;
  private final FieldLayout fieldLayout;

  private boolean initialized = false;

  public Dyn4jCollisionLayer(Mass mass, FieldLayout fieldLayout) {
    this.mass = mass;
    this.fieldLayout = fieldLayout;
    
    this.world = new World<>();
    this.world.setGravity(World.ZERO_GRAVITY);
    this.robotBody = new Body();
  }

  @Override
  public ChassisSpeeds process(Pose2d currentPose, ChassisSpeeds inputSpeeds, Translation2d robotDimensions) {
    if (!initialized) {
      initEnvironment(robotDimensions);
      initialized = true;
    }

    robotBody.getTransform().setTranslation(currentPose.getX(), currentPose.getY());
    robotBody.getTransform().setRotation(currentPose.getRotation().getRadians());

    Translation2d robotRelVel = new Translation2d(inputSpeeds.vxMetersPerSecond, inputSpeeds.vyMetersPerSecond);
    Translation2d fieldRelVel = robotRelVel.rotateBy(currentPose.getRotation());

    robotBody.setLinearVelocity(new Vector2(fieldRelVel.getX(), fieldRelVel.getY()));
    robotBody.setAngularVelocity(inputSpeeds.omegaRadiansPerSecond);

    world.update(0.020);

    Vector2 postLinearVel = robotBody.getLinearVelocity();
    double postAngularVel = robotBody.getAngularVelocity();

    Translation2d postFieldVel = new Translation2d(postLinearVel.x, postLinearVel.y);
    Translation2d postRobotVel = postFieldVel.rotateBy(currentPose.getRotation().unaryMinus());

    return new ChassisSpeeds(postRobotVel.getX(), postRobotVel.getY(), postAngularVel);
  }

  private void initEnvironment(Translation2d robotDimensions) {
    BodyFixture fixture = robotBody.addFixture(
        Geometry.createRectangle(robotDimensions.getX() * 2.0, robotDimensions.getY() * 2.0)
    );
    fixture.setFriction(0.2);
    fixture.setRestitution(0.1);
    
    robotBody.setMass(MassType.NORMAL);
    robotBody.setMass(new org.dyn4j.geometry.Mass(Vector2.create(0, 0), mass.in(Kilograms),0)); // TODO: Fix to be realistic.
    
    world.addBody(robotBody);

    // Inject field obstacles into the physics world
    if (fieldLayout != null) {
      fieldLayout.populateWorld(world);
    }
  }
}