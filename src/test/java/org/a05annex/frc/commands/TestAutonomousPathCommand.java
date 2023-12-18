package org.a05annex.frc.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.DummyScheduledCommand;
import frc.robot.commands.DummyStopAndRunCommand;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.subsystems.DummySwerveDriveSubsystem;
import org.a05annex.frc.subsystems.ISwerveDrive;
import org.a05annex.util.geo2d.KochanekBartelsSpline;
import org.jetbrains.annotations.NotNull;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;

import java.io.File;
import java.io.FileNotFoundException;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;


/**
 * This is a test of the {@link AutonomousPathCommand} that uses a test path
 * with both scheduled commands and stop-and-run commands. The test path is a 5 control point path. Stop-and-run
 * commands happen at the 1st, 3rd, and 5th (last) control points, scheduled commands happens at 2
 * locations on the path.
 * <p>
 * The results of this test are a report of what happens during the command cycles of this test - which
 * requires visual inspection by the person running the test for conformance with expected behaviour. This is
 * obviously not a valid test in the sense that it confirms the code runs as expected - we will work to
 * improve this in the future.
 */
@Suite
public class TestAutonomousPathCommand {

    /**
     *  This override of {@link A05Constants.AutonomousPath} overrides the {@link #load()} function to
     *  account for the code being run in the project test filesystem rather than on the Roborio file
     *  system.
     */
    static class TestAutonomousPath extends A05Constants.AutonomousPath {

        public TestAutonomousPath(@NotNull String pathName, int id, @NotNull String filename) {
            super(pathName, id, filename);
        }

        /**
         * The standard load assumes the code is running on the robot and that it is in a proscribed directory
         * in the robot file system. For testing we are specifying the directory path to the tst file relative
         * to the root project directory, so the code to open the file needs to know that.
         *
         * @throws FileNotFoundException Thrown if the file does not exist or cannot be loaded, details are
         * in the exception message.
         */
        @Override
        public void load() throws FileNotFoundException {
            File file = new File(filename);
            System.out.println(file.getAbsolutePath());
            if (!file.exists()) {
                throw new FileNotFoundException("File: '" + filename + "' does not exist for path" + pathName);
            }
            KochanekBartelsSpline spline = new KochanekBartelsSpline();
            if (!spline.loadPath(filename)) {
                throw new FileNotFoundException("Error loading '" + filename + "' for path" + pathName);
            }
            this.spline = spline;
        }

    }

    /**
     * An override to say this command runs when disabled, so we can use the {@link CommandScheduler} for testing
     * so things are running as on the robot (without a real driver station to connect to, the scheduler decides
     * the robot is disabled, and will not schedule anything.
     */
    class ExtendedAutonomousPathCommand extends AutonomousPathCommand {

        /**
         * Constructor for the {@code AutonomousPathCommand}.
         *
         * @param path                   The path description.
         * @param driveSubsystem         The swerve drive subsystem.
         * @param additionalRequirements Additional required subsystems.
         */
        public ExtendedAutonomousPathCommand(@NotNull A05Constants.AutonomousPath path,
                                             @NotNull ISwerveDrive driveSubsystem,
                                             Subsystem... additionalRequirements) {
            super(path, driveSubsystem, additionalRequirements);
        }

        /**
         * return {@code true} so we can test this thing.
         * @return {@code true}
         */
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }

    private static final double TEST_DRIVE_LENGTH = 0.5969;
    private static final double TEST_DRIVE_WIDTH = 0.5969;

    /**
     *
     */
    @Test
    @DisplayName("Test AutonomousPathCommand")
    void test_autonomousPathCommand() {
        TestAutonomousPath testPath = new TestAutonomousPath("test path",
                0, "./src/test/resources/paths/AutonomousPathCommandTest.json");
        // instantiate the AutonomousPathCommand with the test path and the DummySwerveDriveSubsystem,
        // get a scheduler and schedule the Autonomous
        try {
            testPath.load();
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        DummySwerveDriveSubsystem.getInstance().setDriveGeometry(TEST_DRIVE_LENGTH, TEST_DRIVE_WIDTH,
                0.0, 0.0, 0.0, 0.0, 1.0);
        AutonomousPathCommand autonomousPathCommend = new ExtendedAutonomousPathCommand(
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

        // OK, now that we've run the path, verify that the commands were properly run
        // The DummyScheduledCommand runs twice
        assertEquals(2, DummyScheduledCommand.instantiationCt);
        assertEquals(2, DummyScheduledCommand.initializationCt);
        assertEquals(2, DummyScheduledCommand.endCt);
        assertEquals(2 * DummyScheduledCommand.EXECUTES_PER_SCHEDULED_RUN, DummyScheduledCommand.executeCt);
        // The DummyStopAndRunCommand runs 3 times
        assertEquals(3, DummyStopAndRunCommand.instantiationCt);
        assertEquals(3, DummyStopAndRunCommand.initializationCt);
        assertEquals(3, DummyStopAndRunCommand.endCt);
        assertEquals(3 * DummyStopAndRunCommand.STOP_AND_RUN_DURATION,
                DummyStopAndRunCommand.stopAndRunDuration);
        // and the actual duration may be a bit more/less. The executes should be the
        // cumulative duration / 20 + a couple extra executes because of timing uncertainties, say maybe 10 (.2 sec)
        assertTrue((DummyStopAndRunCommand.stopAndRunDuration / 20) + 10 > DummyStopAndRunCommand.executeCt);
        assertTrue((DummyStopAndRunCommand.stopAndRunDuration / 20) < DummyStopAndRunCommand.executeCt);
    }
}
