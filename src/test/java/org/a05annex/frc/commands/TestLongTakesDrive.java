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

/**
 * This is a test of the autonomous path command for a takes drive command that
 * takes longer than the time to the next control point.
 */
@Suite
public class TestLongTakesDrive {
    @Test
    @DisplayName("Test AutonomousPathCommand - long takes drive command")
    void test_longTakesDriveCommand() {
        A05Constants.setPrintDebug(true);
        TestAutonomousPathCommand.TestAutonomousPath testPath = new TestAutonomousPathCommand.TestAutonomousPath("test path",
                0, "./src/test/resources/paths/TakeDriveCmdLongTest.json");
        // instantiate the AutonomousPathCommand with the test path and the DummySwerveDriveSubsystem,
        // get a scheduler and schedule the Autonomous
        try {
            testPath.load();
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        DummySwerveDriveSubsystem.getInstance().setDriveGeometry(TestAutonomousPathCommand.TEST_DRIVE_LENGTH, TestAutonomousPathCommand.TEST_DRIVE_WIDTH,
                0.0, 0.0, 0.0, 0.0, 1.0);
        AutonomousPathCommand autonomousPathCommend = new TestAutonomousPathCommand.ExtendedAutonomousPathCommand(
                testPath, DummySwerveDriveSubsystem.getInstance());

        long startTime = System.currentTimeMillis();
        System.out.printf("Start time: %d%n", startTime);
        long nextTime = startTime + 20;

        CommandScheduler.getInstance().enable();
        CommandScheduler.getInstance().schedule(autonomousPathCommend);
        while (!autonomousPathCommend.isFinished()) {
            CommandScheduler.getInstance().run();
            try {
                long msSleep = nextTime-System.currentTimeMillis();
                if (msSleep > 0) {
                    //noinspection BusyWait
                    Thread.sleep(nextTime - System.currentTimeMillis());
                }
            } catch (InterruptedException e) {
                break;
            }
            nextTime += 20;
        }

        // The path has been run - there should have been 4 commands that the path attempted to run that were invalid.
        // 2 of these were invalid names, so nothing was instantiated
        assertEquals(0, AutonomousPathCommand.invalidCommandCt);
//        // One of these was a class that is not a Command
//        assertEquals(1, NotCommand.instantiationCt);
//        // One was as a takes drive - but it oes not implement ICanTakeDrive
//        assertEquals(1, DummyScheduledCommand.instantiationCt);
//        assertEquals(0, DummyScheduledCommand.initializationCt);
//        assertEquals(0, DummyScheduledCommand.endCt);
//        assertEquals(0, DummyScheduledCommand.executeCt);
    }
}
