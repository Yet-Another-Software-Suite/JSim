// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import org.junit.jupiter.api.Test;

class FieldDefinitionCatalogTest {
  @Test
  void loadsBuiltInSeasonDefinition() {
    JsonNode field2024 = FieldDefinitionCatalog.loadFieldNode(2024);
    assertEquals(2024, field2024.get("year").asInt());
    assertTrue(field2024.has("field_boundary"));
    assertTrue(field2024.get("field_boundary").has("vertices"));
    assertTrue(field2024.get("field_boundary").get("vertices").size() >= 4);
  }

  @Test
  void exposesAvailableYears() {
    assertTrue(FieldDefinitionCatalog.availableYears().contains(2024));
    assertTrue(FieldDefinitionCatalog.availableYears().contains(2025));
    assertTrue(FieldDefinitionCatalog.availableYears().contains(2026));
  }

  @Test
  void loads2026FieldDefinition() {
    JsonNode field2026 = FieldDefinitionCatalog.loadFieldNode(2026);
    assertEquals(2026, field2026.get("year").asInt());
    assertTrue(field2026.has("field_boundary"));
    assertTrue(field2026.get("field_boundary").has("vertices"));
    assertTrue(field2026.get("field_boundary").get("vertices").size() >= 4);
  }
}
