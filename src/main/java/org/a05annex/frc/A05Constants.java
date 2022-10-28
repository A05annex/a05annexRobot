package org.a05annex.frc;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.a05annex.util.geo2d.KochanekBartelsSpline;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;

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

    private static XboxController.Axis INCREASE_GAIN_AXIS = null,
            DECREASE_GAIN_AXIS = null;

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

    public static XboxController.Axis getIncreaseGainAxis() {
        return INCREASE_GAIN_AXIS;
    }

    public static XboxController.Axis getDecreaseGainAxis() {
        return DECREASE_GAIN_AXIS;
    }

    public static void setGainAxis(XboxController.Axis increaseGain, XboxController.Axis decreaseGain) {
        INCREASE_GAIN_AXIS = increaseGain;
        DECREASE_GAIN_AXIS = decreaseGain;
    }


    public static void setDriveOrientationkp(double kp) {
        DRIVE_ORIENTATION_kP = kp;
    }


    // Digital input switchboard
    private static final DigitalInput switch0 = new DigitalInput(4);
    private static final DigitalInput switch1 = new DigitalInput(3);
    private static final DigitalInput switch2 = new DigitalInput(2);
    private static final DigitalInput switch3 = new DigitalInput(1);
    private static final DigitalInput switch4 = new DigitalInput(0);

    public static int readDriverID() {
        return (switch0.get() ? 0 : 1) + (switch1.get() ? 0 : 2);
    }

    public static int readAutoID() {
        return (switch2.get() ? 0 : 1) + (switch3.get() ? 0 : 2) + (switch4.get() ? 0 : 4);
    }

    public static void printIDs() {
        SmartDashboard.putNumber("driver", readDriverID());
        SmartDashboard.putNumber("auto", readAutoID());
    }

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
}
