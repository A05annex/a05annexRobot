package org.a05annex.frc.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;
import org.photonvision.PhotonCamera;

import static org.a05annex.frc.A05Constants.dict;


public class AprilTagPositionCommand extends A05DriveCommand {

    private final SpeedCachedSwerve swerveDrive = SpeedCachedSwerve.getInstance();

    SpeedCachedSwerve.RobotRelativePosition positionAtFrame;

    private final A05Constants.AprilTagPositionParameters positionParameters;
    private final double xPosition, yPosition, maxSpeed, speedSmoothingMultiplier;
    private final AngleD heading;

    private boolean isFinished;

    private final int[] aprilTagIds;

    private final PhotonCameraWrapper camera = new PhotonCameraWrapper(new PhotonCamera(""));

    private final int resumeDrivingTickThreshold = 500;

    private int ticksWithoutTarget;

    private final int TICKS_IN_ZONE = 10;
    private int ticksInZoneCounter;

    private final double inZoneThreshold;

    private final double X_MAX, X_MIN, Y_MAX, Y_MIN;

    // Constants
    private final double MAX_SPEED_DELTA = 0.075, ROTATION_KP = 0.9;

    public AprilTagPositionCommand(XboxController xbox, A05Constants.DriverSettings driver,
                                   double xPosition, double yPosition, String positionParametersKey) {
        // NOTE: the super adds the drive subsystem requirement
        super(SpeedCachedSwerve.getInstance(), xbox, driver);

        this.positionParameters = dict.get(positionParametersKey);

        this.xPosition = xPosition;
        this.yPosition = -yPosition;
        this.maxSpeed = positionParameters.maxSpeed;
        this.speedSmoothingMultiplier = positionParameters.speedSmoothingMultiplier;
        this.heading = positionParameters.heading;
        this.aprilTagIds = positionParameters.tagIDs;
        this.X_MIN = positionParameters.X_MIN;
        this.X_MAX = positionParameters.X_MAX;
        this.Y_MIN = positionParameters.Y_MIN;
        this.Y_MAX = positionParameters.Y_MAX;

        if(xPosition >= 1) {
            inZoneThreshold = Units.inchesToMeters(0.5 * xPosition);
        } else {
            inZoneThreshold = Units.inchesToMeters(0.5 * Math.pow(xPosition, 2.5));
        }
    }

    @Override
    public void initialize() {
        camera.updateLatestFrameAndTarget();
        /*
          Is there a good target?
          Yes: set ticksWithoutTarget to 0
          No: set ticksWithoutTarget to the resumeDrivingTickThreshold because that is the point at which we resume normal driver control

          If there is not immediately a target, the driver can keep going until there is a target, which means the
          robot won't randomly stop meaning we move faster and smoother
        */
        ticksWithoutTarget = camera.doesLatestFrameAndTargetMatch() ? 0 : resumeDrivingTickThreshold;
        ticksInZoneCounter = 0;
        isFinished = false;

        aprilTagIds = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true) ? aprilTagSet.red : aprilTagSet.blue;

        //heading = upfield ? navX.getHeadingInfo().getClosestUpField() : navX.getHeadingInfo().getClosestDownField();
    }

    @Override
    public void execute() {
        // Update the last frame and related values
        camera.updateLatestFrameAndTarget();


        boolean goodID = false;

        for(int aprilTagId : aprilTagIds) {
            if(aprilTagId == camera.getLatestTarget().getFiducialId()) {
                goodID = true;
            }
        }

        if(!goodID) {
            if(driveXbox == null) {
                isFinished = true;
                return;
            }
            super.execute();
            return;
        }

        /*
        Is there a new target? (can the camera still pickup an aprilTag)

        No: increment ticksWithoutTarget and if we haven't had a new target for a while resume joystick driving
        */
        if(!camera.doesLatestFrameAndTargetMatch()) {
            ticksWithoutTarget++;
            if(ticksWithoutTarget > resumeDrivingTickThreshold) {
                // We haven't had a target for a while. we are going to resume driver control
                if(driveXbox == null) {
                    isFinished = true;
                    return;
                }
                super.execute();
                return;
            }
        } else {
            ticksWithoutTarget = 0;
        }

        positionAtFrame = swerveDrive.getRobotRelativePositionSince(camera.getLatestFrameWithTarget().getTimestampSeconds());
        if(positionAtFrame.cacheOverrun) {
            isFinished = true;
            return;
        }

        // --------- Calculate Speed ---------
        //double totalSpeed = Math.pow(Math.abs(calcX()), speedSmoothingMultiplier) + Math.pow(Math.abs(calcY()), speedSmoothingMultiplier);
        double totalSpeed = calcSpeed();

        // Limit the speed delta
        conditionedSpeed = Utl.clip(totalSpeed, lastConditionedSpeed - MAX_SPEED_DELTA, lastConditionedSpeed + MAX_SPEED_DELTA);

        // Slows the robot down as we go longer without a target. Hopefully allows the robot to "catch" the target again
        conditionedSpeed *= ((double) (resumeDrivingTickThreshold - ticksWithoutTarget) / (double) resumeDrivingTickThreshold);

        // Clip robot speed to stay below max speed
        conditionedSpeed = Utl.clip(conditionedSpeed, 0.0, maxSpeed);


        // ------- Calculate Rotation --------
        // find heading at specified field angle closest to current heading
        AngleD fieldHeading = navX.getHeadingInfo().getClosestHeading(heading.getDegrees());
        navX.setExpectedHeading(fieldHeading);
        conditionedRotate = new AngleD(navX.getHeadingInfo().expectedHeading).subtract(new AngleD(navX.getHeadingInfo().heading))
                .getRadians() * ROTATION_KP;


        // ------- Calculate Direction -------
        conditionedDirection.atan2(calcY(), calcX());

        // Add heading offset
        conditionedDirection.add(heading);


        // Update lasts
        lastConditionedDirection = conditionedDirection;
        lastConditionedSpeed = conditionedSpeed;
        lastConditionedRotate = conditionedRotate;

        // Check if the robot is in the zone
        if(Math.abs(camera.getXFromLastTarget() - positionAtFrame.forward - xPosition) < inZoneThreshold && Math.abs(camera.getYFromLastTarget() - positionAtFrame.strafe - yPosition) < inZoneThreshold) {
            ticksInZoneCounter++;
            swerveDrive.swerveDrive(AngleD.ZERO, 0.0, conditionedRotate * 0.1);
            if(ticksInZoneCounter > TICKS_IN_ZONE) {
                isFinished = true;
            }
            return;
        }

        swerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the swerve
        swerveDrive.swerveDrive(AngleConstantD.ZERO, 0.0, 0.0);
    }

    private double calcSpeed() {
        return Math.pow(Math.sqrt(Math.pow(Math.abs(calcX()), 2) + Math.pow(Math.abs(calcY()), 2)), speedSmoothingMultiplier);
    }

    /**
     * Calculates the X speed between -1 and 1
     *
     * @return x speed
     */
    private double calcX() {
        double center = (X_MAX + X_MIN) / 2.0;
        double scale = (X_MAX - X_MIN) / 2.0;
        return Utl.clip((camera.getXFromLastTarget() - positionAtFrame.forward - center) / scale - (xPosition - center) / scale, -1.0, 1.0);
    }

    /**
     * Calculates the Y speed between -1 and 1
     *
     * @return y speed
     */
    private double calcY() {
        double center = (Y_MAX + Y_MIN) / 2.0;
        double scale = (Y_MAX - Y_MIN) / 2.0;
        return Utl.clip((camera.getYFromLastTarget() - positionAtFrame.strafe - center) / scale - (yPosition - center) / scale, -1.0, 1.0);
    }
}
