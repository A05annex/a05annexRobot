package org.a05annex.frc;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.a05annex.util.geo2d.KochanekBartelsSpline;
import static org.a05annex.util.JsonSupport.*;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.json.simple.JSONObject;
import org.json.simple.parser.ParseException;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.PrimitiveIterator;

/**
 * This is the default constants class for our swerve drive base with NavX, and switch selection of driver
 * configuration and autonomous configurations
 */
public abstract class A05Constants {

    // ---------------------
    // Does this robot have cameras that need to be initialized, and what are they
    /**
     *
     */
    private static boolean HAS_LIMELIGHT = false;
    /**
     *
     */
    private static boolean HAS_USB_CAMERA = false;

    /**
     * @param hasUSB
     * @param hasLimelight
     */
    @SuppressWarnings("unused")
    public static void setCameras(boolean hasUSB, boolean hasLimelight) {
        HAS_USB_CAMERA = hasUSB;
        HAS_LIMELIGHT = hasLimelight;
    }

    /**
     * @return
     */
    @SuppressWarnings("unused")
    public static boolean hasUsbCamera() {
        return HAS_USB_CAMERA;
    }

    /**
     * @return
     */
    @SuppressWarnings("unused")
    public static boolean hasLimelight() {
        return HAS_LIMELIGHT;
    }
    // ---------------------

    public static final int DRIVE_XBOX_PORT = 0;

    // drive encoder tics per radian of robot rotation when rotation is controlled by position rather than speed.
    private static double DRIVE_POS_TICS_PER_RADIAN;

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

    public static double getDrivePosTicsPerRadian() {
        return DRIVE_POS_TICS_PER_RADIAN;
    }

    public static double getDriveOrientationkp() {
        return DRIVE_ORIENTATION_kP;
    }

    public static void setDriveOrientationkp(double kp) {
        DRIVE_ORIENTATION_kP = kp;
    }


    // -----------------------------------------------------------------------------------------------------------------
    // This is the switch panel for autonomous, driver, and robot selection.
    // -----------------------------------------------------------------------------------------------------------------
    // Digital input switchboard
    private static final DigitalInput switch0 = new DigitalInput(4);
    private static final DigitalInput switch1 = new DigitalInput(3);
    private static final DigitalInput switch2 = new DigitalInput(2);
    private static final DigitalInput switch3 = new DigitalInput(1);
    private static final DigitalInput switch4 = new DigitalInput(0);
    private static final DigitalInput switch5 = new DigitalInput(5);

    /**
     * Read and return the driver Id from the switch panel.
     * @return The driver Id, {@code 0} if the switch panel is not connected.
     */
    public static int readDriverID() {
        return (switch0.get() ? 0 : 1) + (switch1.get() ? 0 : 2);
    }

    /**
     * Read and return the autonomous path Id from the switch panel.
     * @return The autonomous path Id, {@code 0} if the switch panel is not connected.
     */
    public static int readAutoID() {
        return (switch2.get() ? 0 : 1) + (switch3.get() ? 0 : 2) + (switch4.get() ? 0 : 4);
    }

    /**
     * Read and return the robot Id from digital IO 5. Install a jumper on the practice robot
     * so the input reads {@code 1}. On the  competition robot without a jumper, the value will default to {@code 0}
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
    public static class AutonomousPath {
        protected final String m_pathName;
        protected final int m_id;
        protected final String m_filename;
        protected KochanekBartelsSpline m_spline;

        /**
         * Instantiate an autonomous path description.
         *
         * @param pathName The path name, usually related to what this path does as in "left start, shot 1",
         *                 "center start, 4 ball"
         * @param id       The position of this path in the path array, used to check that the path that was retrieved
         *                 was the path asked for by the switches.
         * @param filename The filename of this path. This file is expected to be in the /deploy/paths
         *                 subdirectory of ./src/main in your development project.
         */
        public AutonomousPath(@NotNull String pathName, int id, @NotNull String filename) {
            m_pathName = pathName;
            m_id = id;
            m_filename = filename;
        }

        /**
         * Get the ID of this autonomous path.
         *
         * @return Returns the ID (index in the paths list) of this autonomous path.
         */
        public int getId() {
            return m_id;
        }

        /**
         * Gets the name of this autonomous path, mostly used in messaging.
         *
         * @return Returns the name of this path.
         */
        @NotNull
        public String getName() {
            return m_pathName;
        }

        /**
         * Get the filename where this path is stored.
         *
         * @return Returns the filename for this path.
         */
        @NotNull
        public String getFilename() {
            return m_filename;
        }

        /**
         * The loaded spline. If this is called before {@link #load()}, or the load fails, this will be {@code null}.
         *
         * @return Returns the spline for this path.
         */
        @Nullable
        public KochanekBartelsSpline getSpline() {
            return m_spline;
        }

        /**
         * Load this autonomous path on the robot.
         *
         * @throws FileNotFoundException Thrown if the spline file could not be loaded.
         */
        public void load() throws FileNotFoundException {
            KochanekBartelsSpline spline = new KochanekBartelsSpline();
            String filePath = Filesystem.getDeployDirectory() + "/paths/" + m_filename;
            if (!spline.loadPath(filePath)) {
                throw new FileNotFoundException("Could not load file '" + filePath + "' for path" + m_pathName);
            }
            m_spline = spline;
        }
    }

    public static final List<AutonomousPath> AUTONOMOUS_PATH_LIST = new ArrayList<>();

    // -----------------------------------------------------------------------------------------------------------------
    // This is the driver selection stuff
    // -----------------------------------------------------------------------------------------------------------------
    public static class DriverSettings{
        private static final String DRIVE_DEADBAND = "DRIVE_DEADBAND";
        private static final String DRIVE_SPEED_SENSITIVITY = "DRIVE_SPEED_SENSITIVITY";
        private static final String DRIVE_SPEED_GAIN = "DRIVE_SPEED_GAIN";
        private static final String ROTATE_DEADBAND = "ROTATE_DEADBAND";
        private static final String ROTATE_SENSITIVITY = "ROTATE_SENSITIVITY";
        private static final String ROTATE_GAIN = "ROTATE_GAIN";
        private static final String BOOST_GAIN = "BOOST_GAIN";
        private static final String SLOW_GAIN = "SLOW_GAIN";
        private static final String BOOST_TRIGGER = "BOOST_TRIGGER";
        private static final String SLOW_TRIGGER = "SLOW_TRIGGER";
        private static final String LEFT_TRIGGER = "LEFT";
        private static final String RIGHT_TRIGGER = "RIGHT";

        protected double m_driveDeadband, m_driveSpeedSensitivity, m_driveSpeedGain, m_rotateDeadband, m_rotateSensitivity,
                m_rotateGain, m_boostGain, m_slowGain;
        protected XboxController.Axis m_boostTrigger, m_slowTrigger;

        protected final String m_driverName;
        protected final int m_id;

        public DriverSettings(@NotNull String driverName, int id) {
            m_driverName = driverName;
            m_id = id;
        }

        public void load() {
            try {
                String filePath = Filesystem.getDeployDirectory().toString() + "/drivers/" + m_driverName + ".json";
                JSONObject dict = readJsonFileAsJSONObject(filePath);
                if (dict != null) {
                    // Read in the driver data
                    m_driveDeadband = (double)dict.get(DRIVE_DEADBAND);
                    m_driveSpeedSensitivity = (double)dict.get(DRIVE_SPEED_SENSITIVITY);
                    m_driveSpeedGain = (double)dict.get(DRIVE_SPEED_GAIN);
                    m_rotateDeadband = (double)dict.get(ROTATE_DEADBAND);
                    m_rotateSensitivity = (double)dict.get(ROTATE_SENSITIVITY);
                    m_rotateGain = (double)dict.get(ROTATE_GAIN);
                    m_boostGain = (double)dict.get(BOOST_GAIN);
                    m_slowGain = (double)dict.get(SLOW_GAIN);
                    m_boostTrigger = ((String)dict.get(BOOST_TRIGGER)).equals(LEFT_TRIGGER) ?
                            XboxController.Axis.kLeftTrigger : XboxController.Axis.kRightTrigger;
                    m_slowTrigger = ((String)dict.get(SLOW_TRIGGER)).equals(LEFT_TRIGGER) ?
                            XboxController.Axis.kLeftTrigger : XboxController.Axis.kRightTrigger;
                }

            } catch (IOException | ParseException | ClassCastException | NullPointerException e) {
                e.printStackTrace();
                throw new RuntimeException("Driver " + m_driverName + " could not be loaded");
            }
        }

        public double getDriveDeadband() {
            return m_driveDeadband;
        }
        public double getDriveSpeedSensitivity() {
            return m_driveSpeedSensitivity;
        }
        public double getDriveSpeedGain() {
            return m_driveSpeedGain;
        }
        public double getRotateDeadband() {
            return m_rotateDeadband;
        }
        public double getRotateSensitivity() {
            return m_rotateSensitivity;
        }
        public double getRotateGain() {
            return m_rotateGain;
        }
        public double getBoostGain() {
            return m_boostGain;
        }
        public double getSlowGain() {
            return m_slowGain;
        }
        public XboxController.Axis getBoostTrigger() {
            return m_boostTrigger;
        }
        public XboxController.Axis getSlowTrigger() {
            return m_slowTrigger;
        }

        public String getName() {
            return m_driverName;
        }
        public int getId() {
            return m_id;
        }
    }

    public static final List<DriverSettings> DRIVER_SETTINGS_LIST = new ArrayList<>();
}
