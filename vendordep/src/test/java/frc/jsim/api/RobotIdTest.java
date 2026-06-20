package frc.jsim.api;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class RobotIdTest {

    @Test
    void red1_hasCorrectAllianceAndStation() {
        assertEquals(RobotId.Alliance.RED, RobotId.RED_1.alliance);
        assertEquals(1, RobotId.RED_1.station);
    }

    @Test
    void red2_hasCorrectAllianceAndStation() {
        assertEquals(RobotId.Alliance.RED, RobotId.RED_2.alliance);
        assertEquals(2, RobotId.RED_2.station);
    }

    @Test
    void red3_hasCorrectAllianceAndStation() {
        assertEquals(RobotId.Alliance.RED, RobotId.RED_3.alliance);
        assertEquals(3, RobotId.RED_3.station);
    }

    @Test
    void blue1_hasCorrectAllianceAndStation() {
        assertEquals(RobotId.Alliance.BLUE, RobotId.BLUE_1.alliance);
        assertEquals(1, RobotId.BLUE_1.station);
    }

    @Test
    void blue3_hasCorrectAllianceAndStation() {
        assertEquals(RobotId.Alliance.BLUE, RobotId.BLUE_3.alliance);
        assertEquals(3, RobotId.BLUE_3.station);
    }

    @Test
    void sixTotalValues() {
        assertEquals(6, RobotId.values().length);
    }

    @Test
    void of_red2_returnsRed2() {
        assertSame(RobotId.RED_2, RobotId.of(RobotId.Alliance.RED, 2));
    }

    @Test
    void of_blue1_returnsBlue1() {
        assertSame(RobotId.BLUE_1, RobotId.of(RobotId.Alliance.BLUE, 1));
    }

    @Test
    void of_blue3_returnsBlue3() {
        assertSame(RobotId.BLUE_3, RobotId.of(RobotId.Alliance.BLUE, 3));
    }

    @Test
    void of_invalidStation_throwsIllegalArgument() {
        assertThrows(IllegalArgumentException.class,
            () -> RobotId.of(RobotId.Alliance.RED, 4));
        assertThrows(IllegalArgumentException.class,
            () -> RobotId.of(RobotId.Alliance.BLUE, 0));
    }

    @Test
    void toString_formatsCorrectly() {
        assertEquals("Red1", RobotId.RED_1.toString());
        assertEquals("Red2", RobotId.RED_2.toString());
        assertEquals("Red3", RobotId.RED_3.toString());
        assertEquals("Blue1", RobotId.BLUE_1.toString());
        assertEquals("Blue2", RobotId.BLUE_2.toString());
        assertEquals("Blue3", RobotId.BLUE_3.toString());
    }

    @Test
    void redAndBlueAlliancesAreDistinct() {
        assertNotSame(RobotId.RED_1, RobotId.BLUE_1);
        assertNotEquals(RobotId.RED_1.alliance, RobotId.BLUE_1.alliance);
    }

    @Test
    void allValues_haveValidStations() {
        for (RobotId id : RobotId.values()) {
            assertTrue(id.station >= 1 && id.station <= 3,
                "Station out of range for " + id);
        }
    }
}
