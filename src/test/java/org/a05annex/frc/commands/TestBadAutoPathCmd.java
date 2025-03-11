package org.a05annex.frc.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DummyScheduledCommand;
import frc.robot.commands.NotCommand;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.subsystems.DummySwerveDriveSubsystem;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;

import java.io.FileNotFoundException;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * This is a test of a path that has bad class names for robot actions, or specifies robot action classes
 * that do not exist, are not commands, or do not implement required interfaces.
 */
@Suite
public class TestBadAutoPathCmd {
    @Test
    @DisplayName("Test AutonomousPathCommand - bad command classes")
    void test_badActionCommandName() {
        TestAutonomousPathCommand.runAutonomousPath(
                "Test for bad robot action command specification",
                "./src/test/resources/paths/BadCmdClasses.json",
                4, 0, -1, 0);
        // One of these was a class that is not a Command
        assertEquals(1, NotCommand.instantiationCt);
        // One was as a takes drive - but it does not implement ICanTakeDrive
        assertEquals(1, DummyScheduledCommand.instantiationCt);
        assertEquals(0, DummyScheduledCommand.initializationCt);
        assertEquals(0, DummyScheduledCommand.endCt);
        assertEquals(0, DummyScheduledCommand.executeCt);
    }
}
