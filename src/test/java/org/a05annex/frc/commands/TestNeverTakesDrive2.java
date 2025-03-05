package org.a05annex.frc.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DummyStopAndRunCommand;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.subsystems.DummySwerveDriveSubsystem;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;

import java.io.FileNotFoundException;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

@Suite
public class TestNeverTakesDrive2 {
    @Test
    @DisplayName("Test AutonomousPathCommand - never takes drive command, 2 cycles")
    void test_neverTakesDriveCommand2() {
        AutonomousPathCommand.zeroCounts();
        DummyStopAndRunCommand.zeroCounts();
        A05Constants.setPrintDebug(true);
        TestAutonomousPathCommand.runAutonomousPath("never takes drive - 2 target and shoot",
                "./src/test/resources/paths/TakesDriveCmdNeverTakesDrive2.json");

        // The path has been run, check what actually happened.
        assertEquals(0, AutonomousPathCommand.invalidCommandCt);
        // Info about stop-and-run commands (1@0.1sec and 1@0.3sec = 2@.4sec), or
        assertEquals(2, DummyStopAndRunCommand.instantiationCt);
        assertEquals(2, DummyStopAndRunCommand.endCt);
        assertTrue((DummyStopAndRunCommand.stopAndRunDuration / 20) + 2 > DummyStopAndRunCommand.executeCt);
        assertTrue((DummyStopAndRunCommand.stopAndRunDuration / 20) <= DummyStopAndRunCommand.executeCt);
//        // One of these was a class that is not a Command
//        assertEquals(1, NotCommand.instantiationCt);
//        // One was as a takes drive - but it oes not implement ICanTakeDrive
//        assertEquals(1, DummyScheduledCommand.instantiationCt);
//        assertEquals(0, DummyScheduledCommand.initializationCt);
//        assertEquals(0, DummyScheduledCommand.endCt);
//        assertEquals(0, DummyScheduledCommand.executeCt);
    }
}
