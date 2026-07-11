package jsim.field;

import edu.wpi.first.math.geometry.Translation2d;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import jsim.field.SeasonFieldSpec.ColliderSpec;
import jsim.field.SeasonFieldSpec.FieldElement;
import jsim.field.SeasonFieldSpec.FieldSize;
import jsim.field.SeasonFieldSpec.MaterialSpec;
import jsim.field.SeasonFieldSpec.PoseSpec;

/**
 * Reflection-based ingestion for season {@code FieldConstants} classes.
 *
 * <p>This supports a minimal common subset found in Mechanical-Advantage style constants:
 * top-level {@code fieldLength}/{@code fieldWidth} and nested classes that expose
 * {@code nearLeftCorner}, {@code farRightCorner}, and {@code height}.
 */
public final class FieldConstantsSeasonSpecIngestor {
    private FieldConstantsSeasonSpecIngestor() {}

    public static SeasonFieldSpec fromClassName(String className, String seasonId) {
        try {
            return fromClass(Class.forName(className), seasonId);
        } catch (ClassNotFoundException e) {
            throw new IllegalArgumentException("Could not find FieldConstants class: " + className, e);
        }
    }

    public static SeasonFieldSpec fromClass(Class<?> fieldConstantsClass, String seasonId) {
        double fieldLength = readStaticDouble(fieldConstantsClass, "fieldLength");
        double fieldWidth = readStaticDouble(fieldConstantsClass, "fieldWidth");
        int year = inferYear(fieldConstantsClass.getPackageName());

        List<FieldElement> elements = new ArrayList<>();
        String[] knownElementNames = {
                "Hub", "LeftBump", "RightBump", "LeftTrench", "RightTrench", "Tower", "Outpost", "Depot", "FuelPool"
        };

        for (String nested : knownElementNames) {
            Class<?> nestedClass = findNested(fieldConstantsClass, nested);
            if (nestedClass == null) continue;
            tryAddCornerBox(elements, nestedClass);
        }

        return new SeasonFieldSpec(
                1,
                seasonId,
                year,
                "",
                new FieldSize(fieldLength, fieldWidth),
                elements,
                List.of(),
                List.of(),
                Map.of("ingestedFrom", fieldConstantsClass.getName()));
    }

    private static void tryAddCornerBox(List<FieldElement> elements, Class<?> nestedClass) {
        Field nearLeftF = findField(nestedClass, "nearLeftCorner");
        Field farRightF = findField(nestedClass, "farRightCorner");
        Field heightF = findField(nestedClass, "height");
        if (nearLeftF == null || farRightF == null || heightF == null) {
            return;
        }
        try {
            Translation2d nearLeft = (Translation2d) nearLeftF.get(null);
            Translation2d farRight = (Translation2d) farRightF.get(null);
            double height = heightF.getDouble(null);

            double minX = Math.min(nearLeft.getX(), farRight.getX());
            double maxX = Math.max(nearLeft.getX(), farRight.getX());
            double minY = Math.min(farRight.getY(), nearLeft.getY());
            double maxY = Math.max(farRight.getY(), nearLeft.getY());
            double cx = (minX + maxX) / 2.0;
            double cy = (minY + maxY) / 2.0;
            double halfX = (maxX - minX) / 2.0;
            double halfY = (maxY - minY) / 2.0;
            double halfZ = height / 2.0;

            if (halfX <= 0.0 || halfY <= 0.0 || halfZ <= 0.0) return;

            elements.add(new FieldElement(
                    nestedClass.getSimpleName(),
                    "fieldConstantBox",
                    PoseSpec.of(cx, cy, halfZ),
                    ColliderSpec.box(halfX, halfY, halfZ),
                    MaterialSpec.wall(),
                    false,
                    List.of(),
                    Map.of("sourceNestedClass", nestedClass.getName())));
        } catch (IllegalAccessException ignored) {
            // Skip malformed classes without failing full ingest.
        }
    }

    private static Field findField(Class<?> cls, String name) {
        try {
            return cls.getField(name);
        } catch (NoSuchFieldException e) {
            return null;
        }
    }

    private static Class<?> findNested(Class<?> root, String simpleName) {
        for (Class<?> nested : root.getClasses()) {
            if (nested.getSimpleName().equals(simpleName)) return nested;
        }
        return null;
    }

    private static double readStaticDouble(Class<?> cls, String fieldName) {
        try {
            return cls.getField(fieldName).getDouble(null);
        } catch (ReflectiveOperationException e) {
            throw new IllegalArgumentException(
                    "FieldConstants class is missing required static double '" + fieldName + "'", e);
        }
    }

    private static int inferYear(String packageName) {
        String digits = packageName.replaceAll("[^0-9]", "");
        if (digits.length() >= 4) {
            try {
                return Integer.parseInt(digits.substring(0, 4));
            } catch (NumberFormatException ignored) {
            }
        }
        return 0;
    }
}
