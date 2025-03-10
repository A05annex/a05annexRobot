package org.a05annex.frc.commands;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;

/**
 * This is a test of the autonomous path command for a takes drive command that
 * takes longer than the time to the next control point.
 */
@Suite
public class TestLongTakesDrive {
    @Test
    @DisplayName("Test AutonomousPathCommand - long takes drive command")
    void test_longTakesDriveCommand() {
        TestAutonomousPathCommand.runAutonomousPath("initial wait, 2 target and shoot - take drive ends before control pt time",
                "./src/test/resources/paths/TakeDriveCmdLongTest.json",
                0, 3, 2, 2);
    }
}
