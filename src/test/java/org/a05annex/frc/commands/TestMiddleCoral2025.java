package org.a05annex.frc.commands;

import frc.robot.commands.DummyStopAndRunCommand;
import org.a05annex.frc.A05Constants;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * This is a path for 2025 competition that revealed a major flaw in the {@link AutonomousPathCommand} logic
 * that broke paths that had multiple
 * {@link org.a05annex.util.geo2d.KochanekBartelsSpline.RobotActionType#STOP_AND_RUN_COMMAND}s with multiple
 * {@link org.a05annex.util.geo2d.KochanekBartelsSpline.RobotActionType#RELINQUISH_DRIVE_TO_COMMAND}s. We maintain
 * this as a real-life example of how user scenarios find examples that are not found in carefully constructed
 * test scenarios.
 */
@Suite
public class TestMiddleCoral2025 {
    @Test
    @DisplayName("Test AutonomousPathCommand - 2025 competition Middle Single Coral path.")
    void test_middleCoral2025() {
        TestAutonomousPathCommand.runAutonomousPath("2025 competition Middle Single Coral path",
                "./src/test/resources/paths/middleSingleCoral.json",
                0, 2, 1, 1);
    }
}
