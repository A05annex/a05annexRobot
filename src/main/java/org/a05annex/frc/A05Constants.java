package org.a05annex.frc;

import edu.wpi.first.wpilibj.XboxController;
import org.a05annex.frc.subsystems.Mk4NeoModule;
import org.a05annex.util.Utl;

public abstract class A05Constants {

    private static boolean HAS_LIMELIGHT, HAS_USB_CAMERA;

    public static final int DRIVE_XBOX_PORT = 0;

    private static XboxController.Axis INCREASE_GAIN_AXIS = null,
            DECREASE_GAIN_AXIS = null;

    // drive encoder tics per radian of robot rotation when rotation is controlled by position rather than speed.
    private static double DRIVE_POS_TICS_PER_RADIAN;

    private static double DRIVE_ORIENTATION_kP;


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
}
