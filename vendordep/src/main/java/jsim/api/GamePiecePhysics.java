// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.api;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Underlying physics state matching standard trajectory model for FlyingGamePiece.
 */
public class GamePiecePhysics {
    /** The linear velocity of the game piece. */
    public Translation3d linearVelocity = new Translation3d();
    /** The angular velocity of the game piece. */
    public Translation3d angularVelocity = new Translation3d();

    /**
     * Applies the Magnus effect to the game piece trajectory.
     */
    public void applyMagnusEffect() {
        // Core physics binding hook layout mapping
    }
}
