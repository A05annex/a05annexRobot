package org.a05annex.frc.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;

import java.io.IOException;

import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.util.WPIUtilJNI;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.photonvision.PhotonCamera;

public class MonitorPhotonCameraWrapper {


    public static void main(String[] args) throws IOException {
        NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
        WPIMathJNI.Helper.setExtractOnStaticLoad(false);
        CameraServerJNI.Helper.setExtractOnStaticLoad(false);

        CombinedRuntimeLoader.loadLibraries(MonitorPhotonCameraWrapper.class, "wpiutiljni", "wpimathjni", "ntcorejni",
                "cscorejni");
        new MonitorPhotonCameraWrapper().run();
    }

    public void run() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("datatable");
        DoubleSubscriber xSub = table.getDoubleTopic("x").subscribe(0.0);
        DoubleSubscriber ySub = table.getDoubleTopic("y").subscribe(0.0);
        inst.startClient4("example client");
        inst.setServer("10.68.31.12"); // where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
        inst.startDSClient(); // recommended if running on DS computer; this gets the robot IP from the DS


        PhotonCameraWrapper CAMERA =
                new PhotonCameraWrapper(new PhotonCamera("Arducam_OV9281_USB_Camera"),
                        0.32, new AngleD(AngleUnit.DEGREES, 24.5));

        while (true) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                System.out.println("interrupted");
                return;
            }
            double x = xSub.get();
            double y = ySub.get();
            System.out.println("X: " + x + " Y: " + y);
        }
    }
}
