package jsim.physics.layers.gamepieces;

import edu.wpi.first.math.geometry.Translation2d;
import org.dyn4j.dynamics.Body;

/**
 * Encapsulates a single Fuel game piece physics body and status.
 */
public class Fuel2026 {

  private final Body body;
  private boolean isIntaked = false;

  public Fuel2026(Body body) {
    this.body = body;
  }

  public Body getBody() {
    return body;
  }

  public boolean isIntaked() {
    return isIntaked;
  }

  public void setIntaked(boolean intaked) {
    this.isIntaked = intaked;
  }

  public Translation2d getPosition() {
    return new Translation2d(
        body.getTransform().getTranslationX(),
        body.getTransform().getTranslationY()
    );
  }
}