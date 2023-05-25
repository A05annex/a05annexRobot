package org.a05annex.frc;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.a05annex.util.geo2d.KochanekBartelsSpline;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.json.simple.JSONObject;
import org.json.simple.parser.ParseException;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.a05annex.util.JsonSupport.readJsonFileAsJSONObject;

/**
 * This is the default constants class for our swerve drive base with NavX, and switch selection of driver
 * configuration and autonomous configurations. Override this class to build the constants for your robot project.
 */
public abstract class A05Constants {

    // ---------------------
    // Does this robot have cameras that need to be initialized, and what are they
    /**
     * Set to {@code true} if the robot has a limelight camera, {@code false} otherwise.
     */
    private static boolean HAS_LIMELIGHT = false;
    /**
     * Set to {@code true} if the robot has a USB camera, {@code false} otherwise.
     */
    private static boolean HAS_USB_CAMERA = false;

    /**
     * Sets the cameras for the robot.
     *
     * @param hasUSB       {@code true} if the robot has a USB camera, {@code false} otherwise.
     * @param hasLimelight {@code true} if the robot has a limelight camera, {@code false} otherwise.
     */
    @SuppressWarnings("unused")
    public static void setCameras(boolean hasUSB, boolean hasLimelight) {
        HAS_USB_CAMERA = hasUSB;
        HAS_LIMELIGHT = hasLimelight;
    }

    /**
     * Query whether this robot has a USB camera
     *
     * @return {@code true} if the robot has a USB camera, {@code false} otherwise.
     */
    @SuppressWarnings("unused")
    public static boolean hasUsbCamera() {
        return HAS_USB_CAMERA;
    }

    /**
     * Query whether this robot has a limelight
     *
     * @return {@code true} if the robot has a limelight camera, {@code false} otherwise.
     */
    @SuppressWarnings("unused")
    public static boolean hasLimelight() {
        return HAS_LIMELIGHT;
    }
    // ---------------------
    /**
     * {@code true} if CAN devices should be set to factory defaults and fully configured from
     * there, {@code false} if it should be assumed configuration is burned into the CAN devices
     * and should be skipped. Defaults to {@code true}.
     */
    private static boolean SPARK_CONFIG_FROM_FACTORY_DEFAULTS = true;
    /**
     * {@code true} if the CAN devices should be flashed after configuration, {@link false} otherwise. Defaults
     * to {@code false}.
     */
    private static boolean SPARK_BURN_CONFIG = false;

    /**
     *
     *
     * @param fromFactoryDefaults Set to {@code true} during pre-competition programming when you are tuning stuff
     *                            and motor configuration may be different with every restart. Once configurations
     *                            have been burned, this should be {@code false} unless you need to tune something.
     * @param burnConfig          Normally {@code false}. Set to {@code true} once you have things tuned, and you
     *                            want to burn settings into the CAN devices as the power-on settings. After you
     *                            have set this to {@code true} and enabled:
     *                            <ul>
     *                            <li>Please verify CAN devices are set as expected;</li>
     *                            <li>Change the robot initialization code to set this to {@code false} and
     *                            {@code fromFactoryDefaults} to {@code false}, and redeploy;</li>
     *                            <li>Power down the robot, then repower the robot;</li>
     *                            <li>Verify the CAN devices are set as expected at power up</li>
     *                            </ul>
     */
    public static void setSparkConfig(boolean fromFactoryDefaults, boolean burnConfig) {
        SPARK_CONFIG_FROM_FACTORY_DEFAULTS = fromFactoryDefaults;
        SPARK_BURN_CONFIG = burnConfig;
    }

    /**
     * Query whether SparkMax configuration should be from factory defaults, or if it should be assumed
     * configuration is burned into the CAN devices and should be skipped.
     *
     * @return {@code true} if CAN devices should be set to factory defaults and fully configured from
     * there, {@code false} if it should be assumed configuration is burned into the CAN devices
     * and should be skipped.
     */
    public static boolean getSparkConfigFromFactoryDefaults() {
        return SPARK_CONFIG_FROM_FACTORY_DEFAULTS;
    }

    /**
     * Query whether SparkMax configuration should be burned after configuration.
     *
     * @return {@code true} if the CAN devices should be flashed after configuration, {@code false} otherwise.
     */
    public static boolean getSparkBurnConfig() {
        return SPARK_BURN_CONFIG;
    }
    // ---------------------
    /**
     *
     */
    public static final int DRIVE_XBOX_PORT = 0;

    private static double DRIVE_ORIENTATION_kP;

    // ---------------------
    // Controlling whether there is debugging logging of this library in the console file for the run.
    /**
     * {@code true} if this a05annexRobot classes should add debugging output to the console,
     * {@code false} otherwise (and by default).
     */
    private static boolean PRINT_DEBUG = false;

    /**
     * Ask whether debugging output should be printed to the console for this a05annexRobot library.
     *
     * @return {@code true} if a05annexRobot logging is enabled, {@code false} otherwise.
     */
    public static boolean getPrintDebug() {
        return PRINT_DEBUG;
    }

    /**
     * Set whether debugging output should be printed to the console for this a05annexRobot library.
     *
     * @param print {@code true} if a05annexRobot logging should be enabled, {@code false} otherwise.
     */
    @SuppressWarnings("unused")
    public static void setPrintDebug(boolean print) {
        PRINT_DEBUG = print;
    }
    // ---------------------

    /**
     * Get the drive orientation Kp, which is the Kp for the PID loop that keeps the robot on the expected heading
     * when no rotation is being applied during drive.
     *
     * @return The drive orientation Kp.
     */
    public static double getDriveOrientationkp() {
        return DRIVE_ORIENTATION_kP;
    }

    /**
     * Set the drive orientation Kp, which is the Kp for the PID loop that keeps the robot on the expected heading
     * when no rotation is being applied during drive.
     *
     * @param kp The drive orientation Kp.
     */
    @SuppressWarnings("unused")
    public static void setDriveOrientationkp(double kp) {
        DRIVE_ORIENTATION_kP = kp;
    }


    // -----------------------------------------------------------------------------------------------------------------
    // This is the switch panel for autonomous, driver, and robot selection.
    // -----------------------------------------------------------------------------------------------------------------
    // Digital input switchboard
    private static final DigitalInput switch0 = new DigitalInput(0);
    private static final DigitalInput switch1 = new DigitalInput(1);
    private static final DigitalInput switch2 = new DigitalInput(2);
    private static final DigitalInput switch3 = new DigitalInput(3);
    private static final DigitalInput switch4 = new DigitalInput(4);
    private static final DigitalInput switch5 = new DigitalInput(5);

    /**
     * Read and return the driver Id from the switch panel.
     *
     * @return The driver Id, {@code 0} if the switch panel is not connected.
     */
    public static int readDriverID() {
        return (switch1.get() ? 0 : 1) + (switch0.get() ? 0 : 2);
    }

    /**
     * Read and return the autonomous path Id from the switch panel.
     *
     * @return The autonomous path Id, {@code 0} if the switch panel is not connected.
     */
    public static int readAutoID() {
        return (switch4.get() ? 0 : 1) + (switch3.get() ? 0 : 2) + (switch2.get() ? 0 : 4);
    }

    /**
     * Read and return the robot Id from digital IO 5. Install a jumper on the practice robot
     * so the input reads {@code 1}. On the  competition robot without a jumper, the value will default to {@code 0}
     *
     * @return The robot Id, {@code 1} is the practice robot, and {@code 0} is the competition robot.
     */
    public static int readRobotID() {
        return switch5.get() ? 0 : 1;
    }

    /**
     * Print the driver Id, autonomous Id, and robot Id to the smart dashboard.
     */
    @SuppressWarnings("unused")
    public static void printIDs() {
        SmartDashboard.putNumber("driver Id", readDriverID());
        SmartDashboard.putNumber("auto Id", readAutoID());
        SmartDashboard.putNumber("robot Id", readRobotID());
    }

    // -----------------------------------------------------------------------------------------------------------------
    // This is the auto path stuff
    // -----------------------------------------------------------------------------------------------------------------

    /**
     * This class is an autonomous path description for the robot. A list of autonomous path descriptions is loaded
     * into {@link #AUTONOMOUS_PATH_LIST}. The autonomous path switches select the path that will be loaded into
     * the {@link org.a05annex.frc.commands.AutonomousPathCommand} by the {@link A05RobotContainer} constructor.
     */
    public static class AutonomousPath {
        /**
         * The path name, usually related to what this path does as in "left start, shoot 1",
         * "center start, shoot 4". Mostly used for visual feedback of the selected autonomous in
         * the smart dashboard, or, for error messaging.
         */
        protected final String pathName;
        /**
         * The path Id. This is the expected index of the path description in the {@link #AUTONOMOUS_PATH_LIST}. If
         * the Id does not match the expected position, then there is a problem in the declaration of the
         * autonomous descriptions.
         */
        protected final int id;
        /**
         * The filename. This file is expected to be in the /deploy/paths
         * subdirectory of ./src/main in your development project.
         */
        protected final String filename;
        /**
         * The spline loaded from the file specified in {@link #filename}. {@code null} if the spline was not, or could not be loaded.
         */
        protected KochanekBartelsSpline spline;

        /**
         * Instantiate an autonomous path description.
         *
         * @param pathName The path name, usually related to what this path does as in "left start, shoot 1",
         *                 "center start, shoot 4"
         * @param id       The position of this path in the path array, used to check that the path that was retrieved
         *                 was the path asked for by the switches.
         * @param filename The filename of this path. This file is expected to be in the /deploy/paths
         *                 subdirectory of ./src/main in your development project.
         */
        public AutonomousPath(@NotNull String pathName, int id, @NotNull String filename) {
            this.pathName = pathName;
            this.id = id;
            this.filename = filename;
        }

        /**
         * Get the ID of this autonomous path.
         *
         * @return Returns the ID (index in the paths list) of this autonomous path.
         */
        @SuppressWarnings("unused")
        public int getId() {
            return id;
        }

        /**
         * Gets the name of this autonomous path, mostly used in messaging.
         *
         * @return Returns the name of this path.
         */
        @NotNull
        public String getName() {
            return pathName;
        }

        /**
         * Get the filename where this path is stored.
         *
         * @return Returns the filename for this path.
         */
        @SuppressWarnings("unused")
        @NotNull
        public String getFilename() {
            return filename;
        }

        /**
         * The loaded spline. If this is called before {@link #load()}, or the load fails, this will be {@code null}.
         *
         * @return Returns the spline for this path.
         */
        @Nullable
        public KochanekBartelsSpline getSpline() {
            return spline;
        }

        /**
         * Load this autonomous path on the robot.
         *
         * @throws FileNotFoundException Thrown if the spline file could not be loaded.
         */
        public void load() throws FileNotFoundException {
            KochanekBartelsSpline spline = new KochanekBartelsSpline();
            String filePath = Filesystem.getDeployDirectory() + "/paths/" + filename;
            if (!spline.loadPath(filePath)) {
                throw new FileNotFoundException("Could not load file '" + filePath + "' for path" + pathName);
            }
            this.spline = spline;
        }
    }

        /**
     * This is the empty list of {@link AutonomousPath} descriptions that should be initialized in the robot
     * {@link A05Robot#robotInit()} override.
     */
    public static final List<AutonomousPath> AUTONOMOUS_PATH_LIST = new ArrayList<>();

    // -----------------------------------------------------------------------------------------------------------------
    // This is the driver selection stuff
    // -----------------------------------------------------------------------------------------------------------------

    /**
     * This class is the description of customized driver settings for a specific driver. A list of driver
     * descriptions is loaded into {@link #DRIVER_SETTINGS_LIST}. The driver switches select the driver settings
     * JSON file that will be loaded into the {@link org.a05annex.frc.commands.A05DriveCommand} by
     * the {@link A05RobotContainer} constructor.
     */
    public static class DriverSettings {
        /**
         * The JSON key for the {@link #driveDeadband}.
         */
        protected static final String DRIVE_DEADBAND = "DRIVE_DEADBAND";
        /**
         * The JSON key for the {@link #driveSpeedSensitivity}.
         */
        protected static final String DRIVE_SPEED_SENSITIVITY = "DRIVE_SPEED_SENSITIVITY";
        /**
         * The JSON key for the {@link #driveSpeedGain}.
         */
        protected static final String DRIVE_SPEED_GAIN = "DRIVE_SPEED_GAIN";
        /**
         * The JSON key for the {@link #driveSpeedMaxInc}.
         */
        protected static final String DRIVE_SPEED_MAX_INC = "DRIVE_SPEED_MAX_INC";
        /**
         * The JSON key for the {@link #rotateDeadband}.
         */
        protected static final String ROTATE_DEADBAND = "ROTATE_DEADBAND";
        /**
         * The JSON key for the {@link #rotateSensitivity}.
         */
        protected static final String ROTATE_SENSITIVITY = "ROTATE_SENSITIVITY";
        /**
         * The JSON key for the {@link #rotateGain}.
         */
        protected static final String ROTATE_GAIN = "ROTATE_GAIN";
        /**
         * The JSON key for the {@link #rotateMaxInc}.
         */
        protected static final String ROTATE_MAX_INC = "ROTATE_MAX_INC";
        /**
         * The JSON key for the {@link #boostGain}.
         */
        protected static final String BOOST_GAIN = "BOOST_GAIN";
        /**
         * The JSON key for the {@link #slowGain}.
         */
        protected static final String SLOW_GAIN = "SLOW_GAIN";
        /**
         * The JSoN key for the {@link #boostTrigger}.
         */
        protected static final String BOOST_TRIGGER = "BOOST_TRIGGER";
        /**
         * The JSON key for the {@link #slowTrigger}.
         */
        protected static final String SLOW_TRIGGER = "SLOW_TRIGGER";
        /**
         * The JSON value specifying the right trigger for either {@link #BOOST_TRIGGER}
         * or {@link #SLOW_TRIGGER}.
         */
        protected static final String LEFT_TRIGGER = "LEFT";
        /**
         * The JSON value specifying the left trigger for either {@link #BOOST_TRIGGER}
         * or {@link #SLOW_TRIGGER}.
         */
        protected static final String RIGHT_TRIGGER = "RIGHT";

        /**
         * The driver name, usually a first name like "Nolan", "Ethan", "Calvin", etc. Mostly used
         * for visual feedback of the selected driver in the smart dashboard, or, for error messaging.
         */
        protected final String driverName;
        /**
         * The driver ID. This is the expected index of the driver description in the {@link #DRIVER_SETTINGS_LIST}. If
         * the ID does not match the expected position, then there is a problem in the declaration of the
         * driver descriptions.
         */
        protected final int id;
        /**
         * The deadband of the drive joystick. This is the distance from {@code 0.0} that is considered to be
         * {@code 0.0}. This accounts for failure of the stick to reliably recenter to {@code 0.0} as well as minor
         * unintentional pressure on the stick.
         */
        protected double driveDeadband;
        /**
         * The linearity (sensitivity) of the speed control. At {@code 1.0}, the stick reading is proportional to the
         * distance from {@code 0.0}. a value greater than {@code 1.0} raises the stick distance to that power,
         * which flattens the requested speed relative to stick position around {@code 0.0}. For example,
         * if the {@code driveSpeedSensitivity = 2.0}, then if you move the stick
         * to a distance of {@code 0.5}, the actual requested speed is {@code 0.5 ^ 2} or {@code 0.25}. Practically,
         * a greater sensitivity means there is more fine control for subtle movements.
         */
        protected double driveSpeedSensitivity;
        /**
         * The maximum speed that will be sent to the drive subsystem during normal driving.
         */
        protected double driveSpeedGain;
        /**
         * The maximum change im speed that can happen in one command cycle (20ms). This is analogous to anti-lock
         * braking systems (ABS) in cars to minimize skidding that generally causes loss of driver control and slower
         * braking or acceleration than could be achieved without skidding. Additionally, reducing
         * skidding helps us predict robot position on the field with greater accuracy. Also, reducing skidding
         * helps minimize damage to the field surface.
         */
        protected double driveSpeedMaxInc;
        /**
         * The deadband of the rotate joystick. This is the distance from {@code 0.0} that is considered to be
         * {@code 0.0}. This accounts for failure of the stick to reliably recenter to {@code 0.0} as well as minor
         * unintentional pressure on the stick.
         */
        protected double rotateDeadband;
        /**
         * The linearity (sensitivity) of the rotation control. At {@code 1.0}, the stick reading is proportional to
         * the distance from {@code 0.0}. a value greater than {@code 1.0} raises the stick distance to that power,
         * which flattens the requested rotation relative to stick position around {@code 0.0}. For example,
         * if the {@code rotateSensitivity = 2.0}, then if you move the stick
         * to a distance of {@code 0.5}, the actual requested rotation is {@code 0.5 ^ 2} or {@code 0.25}. Practically,
         * a greater sensitivity means there is more fine control for subtle movements.
         */
        protected double rotateSensitivity;
        /**
         * The maximum rotation that will be sent to the drive subsystem during normal driving.
         */
        protected double rotateGain;
        /**
         * The maximum change im rotation that can happen in one command cycle (20ms). This is analogous to anti-lock
         * braking systems (ABS) in cars to minimize skidding that generally causes loss of driver control and slower
         * braking or acceleration than could be achieved without skidding. Additionally, reducing
         * skidding helps us predict robot position on the field with greater accuracy. Also, reducing skidding
         * helps minimize damage to the field surface.
         */
        protected double rotateMaxInc;
        /**
         * The trigger (right or left) that initiates a boost in gain. Analogous to a <i>turbo*</i> button. Useful
         * when the robot is doing something really simple, like heading down field, and you just need fast speed
         * with minimal control. {@code null} if boost gain is not enabled.
         */
        protected XboxController.Axis boostTrigger;
        /**
         * The maximum speed that will be sent to the drive subsystem when boost is activated.
         */
        protected double boostGain;
        /**
         * The trigger (right or left) that initiates a reduction in gain. Analogous to a <i>fine control</i> button.
         * Useful when the robot is doing a fine adjustment, and you need fine control of the robot. {@code null}
         * if slow gain is not enabled.
         */
        protected XboxController.Axis slowTrigger;
        /**
         * The maximum speed that will be sent to the drive subsystem when slow is activated.
         */
        protected double slowGain;

        /**
         * Construct a driver settings description.
         *
         * @param driverName The driver name (no spaces please).
         * @param id         The driver index in the {@link #DRIVER_SETTINGS_LIST}.
         */
        public DriverSettings(@NotNull String driverName, int id) {
            this.driverName = driverName;
            this.id = id;
        }

        /**
         * Load this driver's settings from the driver settings file.
         */
        public void load() {
            String filePath = Filesystem.getDeployDirectory() + "/drivers/" + driverName + ".json";
            loadFilePath(filePath);
        }

        /**
         * Load the driver settings from the specified path.
         *
         * @param filePath The path to the settings file.
         */
        protected void loadFilePath(String filePath) {
            try {
                JSONObject dict = readJsonFileAsJSONObject(filePath);
                // Read in the driver data
                driveDeadband = (double) dict.get(DRIVE_DEADBAND);
                driveSpeedSensitivity = (double) dict.get(DRIVE_SPEED_SENSITIVITY);
                driveSpeedGain = (double) dict.get(DRIVE_SPEED_GAIN);
                driveSpeedMaxInc = (double) dict.get(DRIVE_SPEED_MAX_INC);
                rotateDeadband = (double) dict.get(ROTATE_DEADBAND);
                rotateSensitivity = (double) dict.get(ROTATE_SENSITIVITY);
                rotateGain = (double) dict.get(ROTATE_GAIN);
                rotateMaxInc = (double) dict.get(ROTATE_MAX_INC);
                String boostTrigger = (String) dict.get(BOOST_TRIGGER);
                if (null != boostTrigger) {
                    this.boostTrigger = boostTrigger.equals(LEFT_TRIGGER) ?
                            XboxController.Axis.kLeftTrigger : XboxController.Axis.kRightTrigger;
                    boostGain = (double) dict.get(BOOST_GAIN);
                }
                String slowTrigger = (String) dict.get(SLOW_TRIGGER);
                if (null != slowTrigger) {
                    this.slowTrigger = slowTrigger.equals(LEFT_TRIGGER) ?
                            XboxController.Axis.kLeftTrigger : XboxController.Axis.kRightTrigger;
                    slowGain = (double) dict.get(SLOW_GAIN);
                }

            } catch (IOException | ParseException e) {
                e.printStackTrace();
                throw new RuntimeException("Driver " + driverName + " could not be loaded", e);
            }

        }

        /**
         * Save this driver's settings to the driver settings file.
         */
        @SuppressWarnings("unused")
        public void save() {
            String filePath = Filesystem.getDeployDirectory() + "/drivers/" + driverName + ".json";
            saveFilePath(filePath);
        }

        /**
         * Save the driver settings to the specified path.
         *
         * @param filePath The path to the settings file.
         */
        @SuppressWarnings("unchecked")
        protected void saveFilePath(String filePath) {
            JSONObject dict = new JSONObject();
            dict.put(DRIVE_DEADBAND, driveDeadband);
            dict.put(DRIVE_SPEED_SENSITIVITY, driveSpeedSensitivity);
            dict.put(DRIVE_SPEED_GAIN, driveSpeedGain);
            dict.put(DRIVE_SPEED_MAX_INC, driveSpeedMaxInc);
            dict.put(ROTATE_DEADBAND, rotateDeadband);
            dict.put(ROTATE_SENSITIVITY, rotateSensitivity);
            dict.put(ROTATE_GAIN, rotateGain);
            dict.put(ROTATE_MAX_INC, rotateMaxInc);
            if (null != boostTrigger) {
                dict.put(BOOST_TRIGGER,
                        boostTrigger == XboxController.Axis.kLeftTrigger ? LEFT_TRIGGER : RIGHT_TRIGGER);
                dict.put(BOOST_GAIN, boostGain);
            }
            if (null != slowTrigger) {
                dict.put(SLOW_TRIGGER,
                        slowTrigger == XboxController.Axis.kLeftTrigger ? LEFT_TRIGGER : RIGHT_TRIGGER);
                dict.put(SLOW_GAIN, slowGain);
            }

            try (FileWriter file = new FileWriter(filePath)) {
                file.write(dict.toJSONString());
                file.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        /**
         * Gets the name of this driver, mostly used in messaging.
         *
         * @return Returns the name of this driver.
         */
        public String getName() {
            return driverName;
        }

        /**
         * Get the ID of this driver.
         *
         * @return Returns the ID (index in the drivers list) of this driver.
         */
        public int getId() {
            return id;
        }

        /**
         * Get the deadband for the direction/speed stick, see {@link #driveDeadband}.
         *
         * @return Returns the drive deadband.
         */
        public double getDriveDeadband() {
            return driveDeadband;
        }

        /**
         * Get the sensitivity exponent for the direction/speed stick, see {@link #driveSpeedSensitivity}.
         *
         * @return Returns the drive speed sensitivity.
         */
        public double getDriveSpeedSensitivity() {
            return driveSpeedSensitivity;
        }

        /**
         * Get the maximum directional speed, see {@link #driveSpeedGain}.
         *
         * @return Returns the maximum directional speed.
         */
        public double getDriveSpeedGain() {
            return driveSpeedGain;
        }

        /**
         * Get the maximum change in speed per command cycle, see {@link #driveSpeedMaxInc}.
         *
         * @return Returns the maximum change in speed per command cycle.
         */
        public double getDriveSpeedMaxInc() {
            return driveSpeedMaxInc;
        }

        /**
         * Get the dead-band for the rotation stick, see {@link #rotateDeadband}.
         *
         * @return Returns the rotation dead-band.
         */
        public double getRotateDeadband() {
            return rotateDeadband;
        }

        /**
         * Get the sensitivity exponent for the rotation stick, see {@link #rotateSensitivity}.
         *
         * @return Returns the drive rotate sensitivity.
         */
        public double getRotateSensitivity() {
            return rotateSensitivity;
        }

        /**
         * Get the maximum rotational speed, see {@link #rotateGain}.
         *
         * @return Returns the maximum rotational speed.
         */
        public double getRotateGain() {
            return rotateGain;
        }

        /**
         * Get the maximum change in rotation per command cycle, see {@link #rotateMaxInc}.
         *
         * @return Returns the maximum change in rotation per command cycle.
         */
        public double getRotateMaxInc() {
            return rotateMaxInc;
        }

        /**
         * Get the trigger that activates boost gain, see {@link #boostTrigger}.
         *
         * @return Returns the trigger that activates boost gain, {@code null} if boost gain is not an option.
         */
        @Nullable
        public XboxController.Axis getBoostTrigger() {
            return boostTrigger;
        }

        /**
         * The maximum speed when boost is activated, see {@link #boostGain}.
         *
         * @return Returns the maximum speed when boost is activated
         */
        public double getBoostGain() {
            return boostGain;
        }

        /**
         * Get the trigger that activates slow gain, see {@link #slowTrigger}.
         *
         * @return Returns the trigger that activates slow gain, {@code null} if slow gain is not an option.
         */
        @Nullable
        public XboxController.Axis getSlowTrigger() {
            return slowTrigger;
        }

        /**
         * The maximum speed when slow is activated, see {@link #slowGain}.
         *
         * @return Returns the maximum speed when slow is activated.
         */
        public double getSlowGain() {
            return slowGain;
        }
    }

    /**
     * This list of drivers will be loaded in the {@link A05Robot#robotInit()} override. A driver will
     * be loaded from this list as specified by the driver selection switches.
     */
    public static final List<DriverSettings> DRIVER_SETTINGS_LIST = new ArrayList<>();

    // -----------------------------------------------------------------------------------------------------------------
    // This is the multiple robot support stuff (practice robot and competition robot)
    // -----------------------------------------------------------------------------------------------------------------

    /**
     * This class is the description of swerve drive base geometry and calibration. It is used to allow the same
     * code to be used across multiple bases with different geometry and unique calibration values.
     */
    public static class RobotSettings {
        /**
         * The robot Id. This is the expected index of the robot description in the ROBOT_SETTINGS_LIST. If
         * the Id does not match the expected position, then there is a problem in the declaration of the
         * robot descriptions.
         */
        public final int id;
        /**
         * The robot name, usually a functional name like "programming", or "competition". Mostly used
         * for visual feedback of the selected robot in the smart dashboard, or, for error messaging.
         */
        public final String robotName;
        /**
         * The drive length of the robot in meters. Specifically, the distance between the rotation
         * axles of the front a rear wheels.
         */
        public final double length;
        /**
         * The drive width of the robot in meters. Specifically, the distance between the rotation
         * axles of the right a left wheels.
         */
        public final double width;
        /**
         * The absolute position encoder reading for the right-front swerve module when the
         * drive wheel is facing forward.
         */
        public final double rf;
        /**
         * The absolute position encoder reading for the right-rear swerve module when the
         * drive wheel is facing forward.
         */
        public final double rr;
        /**
         * The absolute position encoder reading for the left-front swerve module when the
         * drive wheel is facing forward.
         */
        public final double lf;
        /**
         * The absolute position encoder reading for the left-rear swerve module when the
         * drive wheel is facing forward.
         */
        public final double lr;
        /**
         *
         */
        public final double navxYawCalibration;
        /**
         * The speed calibration factor that corrects from the theoretic maximum speed to
         * the empirically measured maximum speed. This depends on many factors such as wheel
         * surface, field surface, wheel wear, module friction, motor/controller variation,
         * etc., and we have measured different calibration factors for different robots as
         * well a seasonal variations for each robot.
         */
        public final double maxSpeedCalibration;

        /**
         * Instantiate a robot description of a swerve drive base with a specified drive geometry and
         * calibration constants.
         *
         * @param id                  The robot index in the ROBOT_SETTINGS_LIST.
         * @param robotName           The robot name.
         * @param length              (double) The length of the drive in meters.
         * @param width               (double) The width of the drive in meters.
         * @param rf                  (double) The reading of the right front spin absolute position encoder when
         *                            the wheel is facing directly forward.
         * @param rr                  (double) The reading of the right rear spin absolute position encoder when
         *                            the wheel is facing directly forward.
         * @param lf                  (double) The reading of the left front spin absolute position encoder when
         *                            the wheel is facing directly forward.
         * @param lr                  (double) The reading of the left rear spin absolute position encoder when
         *                            the wheel is facing directly forward.
         * @param navxYawCalibration  (double) A calibration factor for the yaw reported by the NavX. We noted
         *                            a repeatable drift per rotation, and measured a correction factor for that.
         * @param maxSpeedCalibration (double) A calibration factor for the swerve module max m/sec to correct the
         *                            msx m/sec computed from all of the spec sheets and mox module motor RPM to
         *                            the empirically measured max m/sec.
         */
        public RobotSettings(int id, String robotName, double length, double width,
                             double rf, double rr, double lf, double lr,
                             double navxYawCalibration, double maxSpeedCalibration) {
            this.id = id;
            this.robotName = robotName;
            this.length = length;
            this.width = width;
            this.rf = rf;
            this.rr = rr;
            this.lf = lf;
            this.lr = lr;
            this.navxYawCalibration = navxYawCalibration;
            this.maxSpeedCalibration = maxSpeedCalibration;
        }
    }

    /**
     * This list of robots will be loaded in the {@link A05Robot#robotInit()} override. A robot will
     * be initialized from this list as specified by robot selection jumper. It is currently anticipated
     * that there will be no more than 2 robots running the same code.
     */
    public static final List<RobotSettings> ROBOT_SETTINGS_LIST = new ArrayList<>();
}
