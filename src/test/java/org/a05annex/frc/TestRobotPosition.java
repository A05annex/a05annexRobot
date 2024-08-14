package org.a05annex.frc;

import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.junit.jupiter.api.Test;

import static org.a05annex.frc.RobotPosition.solveForTruePositionTestMethod;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;

class TestRobotPosition {
    @Test
    void testSolveForTruePosition_whenHeadingDeltaZero() {
        double camX = 5.0;
        double camY = 5.0;
        AngleD headingDelta = new AngleD(AngleUnit.DEGREES, 0.0);

        assertArrayEquals(new double[]{5.0, 5.0}, solveForTruePositionTestMethod(camX, camY, headingDelta), 1e-6);
    }

    @Test
    void testSolveForTruePosition_withNonZeroHeadingDelta() {
        double camX = 3.0;
        double camY = 4.0;
        AngleD headingDelta = new AngleD(AngleUnit.DEGREES, -45.0);

        assertArrayEquals(new double[]{4.9497474683, 0.7071067812}, solveForTruePositionTestMethod(camX, camY, headingDelta), 1e-6);
    }

    @Test
    void testSolveForTruePosition_whenYNegative() {
        double camX = 4.0;
        double camY = -2.5;
        AngleD headingDelta = new AngleD(AngleUnit.DEGREES, -30.0);

        assertArrayEquals(new double[]{2.2141016151, -4.1650635095}, solveForTruePositionTestMethod(camX, camY, headingDelta), 1e-6);
    }

    @Test
    void testSolveForTruePosition_whenHeadingDeltaPositive() {
        double camX = 4.0;
        double camY = 3.0;
        AngleD headingDelta = new AngleD(AngleUnit.DEGREES, 20.0);

        assertArrayEquals(new double[]{2.7327100532, 4.1871584357}, solveForTruePositionTestMethod(camX, camY, headingDelta), 1e-6);
    }

    @Test
    void testSolveForTruePosition_whenHeadingDeltaPositiveAndYNegative() {
        double camX = 4.0;
        double camY = -3.0;
        AngleD headingDelta = new AngleD(AngleUnit.DEGREES, 45.0); // -45 degrees

        assertArrayEquals(new double[]{4.9497474683, 0.7071067812}, solveForTruePositionTestMethod(camX, camY, headingDelta), 1e-6);
    }

    @Test
    void testSolveForTruePosition_randomValues() {
        double camX = 4.05;
        double camY = -2.89;
        AngleD headingDelta = new AngleD(AngleUnit.DEGREES, 12); // -45 degrees

        assertArrayEquals(new double[]{4.5623625694, -1.9848042183}, solveForTruePositionTestMethod(camX, camY, headingDelta), 1e-6);
    }
}
