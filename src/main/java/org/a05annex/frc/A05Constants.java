package org.a05annex.frc;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.a05annex.frc.subsystems.Mk4NeoModule;
import org.a05annex.util.Utl;
import org.a05annex.util.geo2d.KochanekBartelsSpline;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;

public abstract class A05Constants {

    private static boolean HAS_LIMELIGHT, HAS_USB_CAMERA;

    private static boolean PRINT_DEBUG = false;

    public static final int DRIVE_XBOX_PORT = 0;

    private static XboxController.Axis INCREASE_GAIN_AXIS = null,
            DECREASE_GAIN_AXIS = null;

    // drive encoder tics per radian of robot rotation when rotation is controlled by position rather than speed.
    private static double DRIVE_POS_TICS_PER_RADIAN;

    private static double DRIVE_ORIENTATION_kP;

    public static boolean getPrintDebug() {
        return PRINT_DEBUG;
    }

    public static void setPrintDebug(boolean print) {
        PRINT_DEBUG = print;
    }

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


    public static void setCameras(boolean hasUSB, boolean hasLimelight) {
        HAS_USB_CAMERA = hasUSB;
        HAS_LIMELIGHT = hasLimelight;
    }

    public static boolean hasUsbCamera() {
        return HAS_USB_CAMERA;
    }

    public static boolean hasLimelight() {
        return HAS_LIMELIGHT;
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
        private final String m_pathName;
        private final int m_id;
        private final String m_filename;

        private KochanekBartelsSpline m_spline = null;

        public AutonomousPath(String skill, int id, String filename) {
            m_pathName = skill;
            m_id = id;
            m_filename = filename;
        }

        public String getName() {
            return m_pathName;
        }

        public KochanekBartelsSpline getSpline() {
            return m_spline;
        }
        /**
         * Load this autonomous path.
         *
         * @throws FileNotFoundException Thrown if the spline file could not be loaded.
         */
        public void load() throws FileNotFoundException {
            KochanekBartelsSpline spline = new KochanekBartelsSpline();
            if (!spline.loadPath(Filesystem.getDeployDirectory().toString() + "/paths/" +
                    m_filename)) {
                throw new FileNotFoundException("Could not load file '" + Filesystem.getDeployDirectory().toString() + "/paths/" +
                        m_filename + "' for path" + m_pathName);
            }
            m_spline = spline;
        }
    }

    public static final List<AutonomousPath> AUTONOMOUS_PATH_LIST = new ArrayList<>();
}
