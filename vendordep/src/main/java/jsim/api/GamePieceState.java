// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.api;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import jsim.LinearVelocity3d;

/**
 * Represents the state of a game piece in the simulation.
 * Includes type, velocity, rotation, and other physical properties.
 */
public class GamePieceState {
    /** Type of this game piece. */
    public GamePieceType type;
    /** Linear exit velocity used by the current simulation API. */
    public LinearVelocity3d velocity;
    /** Orientation of the piece. */
    public Rotation3d rotation;
    /** Start offset in the robot frame. */
    public Translation2d robotOffsetStart;
    /** Exit velocity vector. */
    public LinearVelocity3d exitVelocity;
    /** Exit orientation. */
    public Rotation3d exitAngle;
    /** Intake polygon in robot space. */
    public Translation3d[] intakeArea;

    /**
     * Creates a game piece state for the given type.
     *
     * @param type game piece type
     */
    public GamePieceState(GamePieceType type) {
        this.type = type;
    }

    /**
     * Returns the piece type.
     *
     * @return the game piece type
     */
    public GamePieceType getType() { return type; }
    /**
        * Sets the linear velocity.
     *
        * @param velocity linear exit velocity
     */
        public void setVelocity(LinearVelocity3d velocity) { this.velocity = velocity; }
    /**
     * Sets the rotation.
     *
     * @param rotation piece rotation
     */
    public void setRotation(Rotation3d rotation) { this.rotation = rotation; }
    /**
    * Returns the linear velocity.
     *
    * @return the linear velocity
     */
    public LinearVelocity3d getVelocity() { return velocity; }
    /**
     * Returns the rotation.
     *
     * @return the piece rotation
     */
    public Rotation3d getRotation() { return rotation; }
    /**
     * Sets the robot offset start.
     *
     * @param offset starting offset in robot coordinates
     */
    public void setRobotOffsetStart(Translation2d offset) { this.robotOffsetStart = offset; }
    /**
     * Returns the robot offset start.
     *
     * @return the starting offset in robot coordinates
     */
    public Translation2d getRobotOffsetStart() { return robotOffsetStart; }
    /**
     * Sets the exit velocity vector.
     *
     * @param velocity exit velocity vector
     */
    public void setExitVelocity(LinearVelocity3d velocity) { this.exitVelocity = velocity; }
    /**
     * Returns the exit velocity vector.
     *
     * @return the exit velocity vector
     */
    public LinearVelocity3d getExitVelocity() { return exitVelocity; }
    /**
     * Sets the exit angle.
     *
     * @param angle exit angle
     */
    public void setExitAngle(Rotation3d angle) { this.exitAngle = angle; }
    /**
     * Returns the exit angle.
     *
     * @return the exit angle
     */
    public Rotation3d getExitAngle() { return exitAngle; }
    /**
     * Sets the intake polygon area.
     *
     * @param intakeArea intake polygon area
     */
    public void intakeZone(Translation3d[] intakeArea) { this.intakeArea = intakeArea; }
    /**
     * Returns the intake polygon area.
     *
     * @return the intake polygon area
     */
    public Translation3d[] getIntakeArea() { return intakeArea; }
}
