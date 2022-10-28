package org.a05annex.frc.commands;

import org.a05annex.frc.A05Constants;
import org.a05annex.frc.subsystems.DummySwerveDriveSubsystem;
import org.a05annex.util.geo2d.KochanekBartelsSpline;
import org.jetbrains.annotations.NotNull;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;

import java.io.File;
import java.io.FileNotFoundException;

/**
 * This is a test of the {@link AutonomousPathCommand} that uses a test path
 * with both scheduled commands and stop-and-run commands. The test path is a 5 control point path. Stop-and-run
 * commands happen at the 1st, 3rd, and 5th (last) control points, scheduled commands happens at 2
 * locations on the path.
 */
@Suite
public class TestAutonomousPathCommand {

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
            File file = new File(m_filename);
            System.out.println(file.getAbsolutePath());
            if (!file.exists()) {
                throw new FileNotFoundException("File: '" + m_filename + "' does not exist for path" + m_pathName);
            }
            KochanekBartelsSpline spline = new KochanekBartelsSpline();
            if (!spline.loadPath(m_filename)) {
                throw new FileNotFoundException("Error loading '" + m_filename + "' for path" + m_pathName);
            }
            m_spline = spline;
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
                0.0, 0.0, 0.0, 0.0);
        AutonomousPathCommand autonomousPathCommend = new AutonomousPathCommand(
                testPath, DummySwerveDriveSubsystem.getInstance());
        autonomousPathCommend.initialize();

        long startTime = System.currentTimeMillis();
        System.out.printf("Start time: %d%n", startTime);
        long nextTime = startTime + 20;
        while (!autonomousPathCommend.isFinished()) {
            autonomousPathCommend.execute();
            try {
                Thread.sleep(nextTime-System.currentTimeMillis());
            } catch (InterruptedException e) {
                break;
            }
            nextTime += 20;
        }
        autonomousPathCommend.end(false);
    }
}
