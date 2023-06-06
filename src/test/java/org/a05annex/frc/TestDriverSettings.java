package org.a05annex.frc;

import edu.wpi.first.wpilibj.XboxController;
import org.jetbrains.annotations.NotNull;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.suite.api.Suite;

import static org.junit.jupiter.api.Assertions.*;

/**
 * This test uses a set of test driver .json settings file to verify error handling, reading, and writing of
 * driver settings files.
 */
@Suite
public class TestDriverSettings {

    /**
     * An extension of {@link A05Constants.DriverSettings} to override the {@link #load()} and add a
     * {@link #save(String)} that map driver names to file paths in the test file system rather than the
     * deployment roborio file system.
     */
    static class DriverSettingsForTest extends A05Constants.DriverSettings {

        /**
         * Construct a driver settings description for testing.
         * @param driverName The driver name (no spaces please).
         * @param id The driver index in the {@link #testDriverSettings} array.
         */
        public DriverSettingsForTest(@NotNull String driverName, int id) {
            super(driverName, id);
        }

        /**
         * Maps the driver name to a settings file in the project test file system (rather than the deploy
         * roborio file system).
         */
        @Override
        public void load() {
            String filePath = "./src/test/resources/drivers/" + driverName + ".json";
            loadFilePath(filePath);
        }

        /**
         * This save lets us save these driver settings to a different driver name so that we can verify that
         * the save actually saves what is in the driver settings by reading what was saved and comparing the read
         * settings to the original settings.
         *
         * @param driverName The driver name for the settings save.
         */
        public void save(String driverName) {
            String filePath = "./src/test/resources/drivers/" + driverName + ".json";
            saveFilePath(filePath);
        }
    }

    A05Constants.DriverSettings[] testDriverSettings = {
                new DriverSettingsForTest("badFileName", 0),
                new DriverSettingsForTest("invalidFormat", 1),
                new DriverSettingsForTest("missingSetting", 2),
                new DriverSettingsForTest("validTest", 3),
                new DriverSettingsForTest("noBoost", 4),
                new DriverSettingsForTest("noSlow", 5),
                new DriverSettingsForTest("saveTest", 6)
        };

    /**
     *
     */
    @Test
    @DisplayName("Test Bad Driver Settings Filename")
    void test_missingFile() {
        assertThrows(RuntimeException.class,
                () -> testDriverSettings[0].load());
    }

    /**
     *
     */
    @Test
    @DisplayName("Test Invalid Driver Settings Format")
    void test_invalidFormat() {
        assertThrows(RuntimeException.class,
                () -> testDriverSettings[1].load());
    }

    /**
     *
     */
    @Test
    @DisplayName("Test Invalid Driver Settings Missing ROTATE_DEADBAND")
    void test_missingSetting() {
        assertThrows(RuntimeException.class,
                () -> testDriverSettings[2].load());
    }

    /**
     *
     */
    @Test
    @DisplayName("Test Driver Settings Read")
    void test_verifyRead() {
        A05Constants.DriverSettings driver = testDriverSettings[3];
        driver.load();
        assertEquals(3, driver.getId());
        assertEquals(0.02, driver.getDriveDeadband());
        assertEquals(2.0, driver.getDriveSpeedSensitivity());
        assertEquals(0.4, driver.getDriveSpeedGain());
        assertEquals(0.065, driver.getDriveSpeedMaxInc());
        assertEquals(0.05, driver.getRotateDeadband());
        assertEquals(1.5, driver.getRotateSensitivity());
        assertEquals(0.45, driver.getRotateGain());
        assertEquals(0.075, driver.getRotateMaxInc());
        assertEquals(XboxController.Axis.kRightTrigger, driver.getBoostTrigger());
        assertEquals(1.0, driver.getBoostGain());
        assertEquals(XboxController.Axis.kLeftTrigger, driver.getSlowTrigger());
        assertEquals(0.3, driver.getSlowGain());
    }

    /**
     *
     */
    @Test
    @DisplayName("Test Driver Settings Read - no boost")
    void test_noBoost() {
        A05Constants.DriverSettings driver = testDriverSettings[4];
        driver.load();
        assertEquals(4, driver.getId());
        assertNull(driver.getBoostTrigger());
        assertEquals(0.0, driver.getBoostGain());
        assertEquals(XboxController.Axis.kLeftTrigger, driver.getSlowTrigger());
        assertEquals(0.3, driver.getSlowGain());
    }

    /**
     *
     */
    @Test
    @DisplayName("Test Driver Settings Read - no slow")
    void test_noSlow() {
        A05Constants.DriverSettings driver = testDriverSettings[5];
        driver.load();
        assertEquals(5, driver.getId());
        assertEquals(XboxController.Axis.kRightTrigger, driver.getBoostTrigger());
        assertEquals(1.0, driver.getBoostGain());
        assertNull(driver.getSlowTrigger());
        assertEquals(0.0, driver.getSlowGain());
    }


    /**
     *
     */
    @Test
    @DisplayName("Test Driver Settings Write")
    void test_verifyWrite() {
        // read the verified driver settings file then save it to the file that will be read by testDriverSettings[6]
        testDriverSettings[3].load();
        ((DriverSettingsForTest)testDriverSettings[3]).save(testDriverSettings[6].getName());
        // Verify that this file is the same as the verified file
        A05Constants.DriverSettings driver = testDriverSettings[6];
        driver.load();
        assertEquals(6, driver.getId());
        assertEquals(0.02, driver.getDriveDeadband());
        assertEquals(2.0, driver.getDriveSpeedSensitivity());
        assertEquals(0.4, driver.getDriveSpeedGain());
        assertEquals(0.065, driver.getDriveSpeedMaxInc());
        assertEquals(0.05, driver.getRotateDeadband());
        assertEquals(1.5, driver.getRotateSensitivity());
        assertEquals(0.45, driver.getRotateGain());
        assertEquals(0.075, driver.getRotateMaxInc());
        assertEquals(XboxController.Axis.kRightTrigger, driver.getBoostTrigger());
        assertEquals(1.0, driver.getBoostGain());
        assertEquals(XboxController.Axis.kLeftTrigger, driver.getSlowTrigger());
        assertEquals(0.3, driver.getSlowGain());
    }

}
