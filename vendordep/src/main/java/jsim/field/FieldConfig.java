// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.field;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

/**
 * Single JSON per season config parser mapping.
 */
//@JsonIgnoreProperties(ignoreUnknown = true)
public class FieldConfig {
    /** The game season associated with this field configuration. */
    public String season;
    /** Array of field elements in this configuration. */
    public FieldElement[] fieldElements;

    /**
     * Creates a new empty FieldConfig with no field elements.
     */
    public FieldConfig() {
        this.fieldElements = new FieldElement[0];
    }
}
