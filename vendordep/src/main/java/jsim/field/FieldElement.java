// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.field;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Basic block of the JSim environment (HUB, DEPOT, HP_STATION).
 */
public class FieldElement {
    /**
     * Create a new empty FieldElement.
     */
    public FieldElement() {}
    /**
     * Enum representing the type of field element.
     */
    public enum Type {
        /** The hub/goal area. */
        HUB,
        /** The depot/supply area. */
        DEPOT,
        /** The human player station. */
        HP_STATION,
        /** An obstacle element. */
        OBSTACLE
    }

    /** The type of this field element. */
    public Type type;
    /** The 3D position of this field element. */
    public Pose3d position;
    // public Composition composition; // Future layout definition structure
}
