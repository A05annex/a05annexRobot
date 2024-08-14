package org.a05annex.frc;

import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.jetbrains.annotations.NotNull;
import org.photonvision.targeting.PhotonPipelineResult;

public class RobotPosition {
    public final boolean isValid;

    public final double x, y;

    public PhotonPipelineResult pipelineResult;

    private static PhotonCameraWrapper camera = null;

    private final static NavX navX = NavX.getInstance();

    private RobotPosition(boolean isValid, double x, double y, PhotonPipelineResult pipelineResult) {
        this.isValid = isValid;
        this.x = x;
        this.y = y;
        this.pipelineResult = pipelineResult;
    }

    private RobotPosition() {
        isValid = false;
        x = 0.0;
        y = 0.0;
        pipelineResult = null;
    }

    public static void setCamera(PhotonCameraWrapper camera) {
        if(camera == null) {
            RobotPosition.camera = camera;
            return;
        }
        throw new Error("Robot Position camera was initialized more than once");
    }

    public static RobotPosition getRobotPosition(String tagSetKey) {
        A05Constants.AprilTagSet tagSet = A05Constants.aprilTagSetDictionary.get(tagSetKey);

        camera.updateTrackingData();

        if(camera.getTarget(tagSet) == null) {
            return new RobotPosition();
        }

        TruePosition pos = solveForTruePosition(tagSet);


        return new RobotPosition(true, pos.x(), pos.y(), camera.getNewestFrameWithTarget());
    }

    private static @NotNull TruePosition solveForTruePosition(A05Constants.AprilTagSet tagSet) {
        double camX = camera.getXFromLastTarget(tagSet); // camera X is distance from target
        double camY = camera.getYFromLastTarget(tagSet); // camera Y is horizontal offset X

        AngleD headingDelta = navX.getHeadingInfo().getClosestHeading(tagSet.heading()).subtract(navX.getHeading()).cloneAngleD(); // Tag heading - current heading

        double[] output = solveForTruePositionTestMethod(camX, camY, headingDelta);

        return new TruePosition(output[0], output[1]);
    }

    static double[] solveForTruePositionTestMethod(double camX, double camY, AngleD headingDelta) {
        headingDelta.mult(-1.0);
        AngleD hypotenuseAngle = headingDelta.add(new AngleD().atan(camX / camY)).cloneAngleD();

        if(camY < 0.0) {
            hypotenuseAngle.add(AngleConstantD.DEG_180);
        }

        double hypotenuse = Math.sqrt(Math.pow(camX, 2) + Math.pow(camY, 2));

        double x = hypotenuseAngle.sin() * hypotenuse; // true X is distance from target
        double y = hypotenuseAngle.cos() * hypotenuse;

        return new double[]{x, y};
    }


    private record TruePosition(double x, double y) {
    }
}
