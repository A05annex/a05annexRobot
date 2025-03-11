package org.a05annex.frc.commands;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;

@Suite
public class TestShortTakesDrive {
    @Test
    @DisplayName("Test AutonomousPathCommand - short takes drive command")
    void test_shortTakesDriveCommand() {
        TestAutonomousPathCommand.runAutonomousPath(
                "initial wait, 2 target and shoot - take drive ends before control pt time",
                "./src/test/resources/paths/TakesDriveCmdShortTest.json",
                0, 3, 2, 2);
    }
}
