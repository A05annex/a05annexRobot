package org.a05annex.frc.commands;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;

@Suite
public class TestNeverTakesDrive {
    @Test
    @DisplayName("Test AutonomousPathCommand - never takes drive command")
    void test_neverTakesDriveCommand() {
        TestAutonomousPathCommand.runAutonomousPath("never takes drive - initial wait, 2 target and shoot",
                "./src/test/resources/paths/TakesDriveCmdNeverTakesDrive2.json",
                0, 3, 2, 2);
   }
}
