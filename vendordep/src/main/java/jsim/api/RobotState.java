// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.api;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Mutable robot snapshot shared between the simulation facade and state manager.
 */
public class RobotState {
    /** Current robot pose on the field. */
    public Pose2d robotPose;
    /** Robot frame perimeter vertices. */
    public final Translation2d[] frameVertices;
    /** Current commanded or measured chassis speeds. */
    public ChassisSpeeds chassisSpeeds;
    /** Robot identifier. */
    public final RobotID id;

    /**
     * Creates a robot state snapshot.
     *
     * @param id robot identifier
     * @param robotPose initial pose
     * @param frameVertices frame perimeter vertices
     */
    public RobotState(RobotID id, Pose2d robotPose, Translation2d[] frameVertices) {
        this.id = id;
        this.robotPose = robotPose;
        this.frameVertices = frameVertices;
        this.chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    }
}
