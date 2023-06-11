package org.a05annex.frc.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.frc.subsystems.SpeedCachedSwerve;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;

import static org.a05annex.frc.A05Constants.aprilTagPositionParametersDictionary;


public class A05AprilTagPositionCommand extends A05DriveCommand {

    private final SpeedCachedSwerve swerveDrive = SpeedCachedSwerve.getInstance();

    SpeedCachedSwerve.RobotRelativePosition positionAtFrame;

    private final A05Constants.AprilTagPositionParameters positionParameters;

    private final PhotonCameraWrapper camera;

    protected boolean canPerformTargeting = false;

    // Counter for how many ticks the robot hasn't seen a valid target
    private int ticksWithoutTarget = 0;

    // Maximum number of ticks where the robot hasn't had a valid target before resuming normal driving
    private final int resumeDrivingTickThreshold = 50;

    // The size of the "box" in which the robot is considered in the right position
    private final double inZoneThreshold;

    // Number of ticks where the robot needs to be in the zone to end the command
    private final int TICKS_IN_ZONE = 10;

    // Counter for how many ticks the robot has been in the zone
    private int ticksInZoneCounter;

    // Drive and control constants
    private final double MAX_SPEED_DELTA = 0.075, ROTATION_KP = 0.9;
    private final double X_POSITION, Y_POSITION, MAX_SPEED, SPEED_SMOOTHING_MULTIPLIER;
    private final int[] aprilTagIds;
    private final AngleD HEADING;
    private final double X_MAX, X_MIN, Y_MAX, Y_MIN;

    private boolean isFinished;

    protected A05AprilTagPositionCommand(XboxController xbox, A05Constants.DriverSettings driver, PhotonCameraWrapper camera,
                                      double xPosition, double yPosition, String positionParametersKey) {
        // NOTE: the super adds the drive subsystem requirement
        super(SpeedCachedSwerve.getInstance(), xbox, driver);

        this.camera = camera;

        this.positionParameters = aprilTagPositionParametersDictionary.get(positionParametersKey);

        this.X_POSITION = xPosition;
        this.Y_POSITION = -yPosition;
        this.MAX_SPEED = positionParameters.maxSpeed;
        this.SPEED_SMOOTHING_MULTIPLIER = positionParameters.speedSmoothingMultiplier;
        this.HEADING = positionParameters.heading;
        this.aprilTagIds = positionParameters.tagIDs;
        this.X_MIN = positionParameters.X_MIN;
        this.X_MAX = positionParameters.X_MAX;
        this.Y_MIN = positionParameters.Y_MIN;
        this.Y_MAX = positionParameters.Y_MAX;

        // When very close to the target, you need a tighter "zone". This statement applies a different formula if
        //          the robot is within 1 meter of the target
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

        // Reset values for the start of a new command
        ticksInZoneCounter = 0;
        isFinished = false;
        canPerformTargeting = false;
    }

    @Override
    public void execute() {
        checkIfCanPerformTargeting();

        executeTargeting();
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
        return Math.pow(Math.sqrt(Math.pow(Math.abs(calcX()), 2) + Math.pow(Math.abs(calcY()), 2)), SPEED_SMOOTHING_MULTIPLIER);
    }

    /**
     * Calculates the X speed between -1 and 1
     *
     * @return x speed
     */
    private double calcX() {
        double center = (X_MAX + X_MIN) / 2.0;
        double scale = (X_MAX - X_MIN) / 2.0;
        return Utl.clip((camera.getXFromLastTarget() - positionAtFrame.forward - center) / scale - (X_POSITION - center) / scale, -1.0, 1.0);
    }

    /**
     * Calculates the Y speed between -1 and 1
     *
     * @return y speed
     */
    private double calcY() {
        double center = (Y_MAX + Y_MIN) / 2.0;
        double scale = (Y_MAX - Y_MIN) / 2.0;
        return Utl.clip((camera.getYFromLastTarget() - positionAtFrame.strafe - center) / scale - (Y_POSITION - center) / scale, -1.0, 1.0);
    }

    protected void checkIfCanPerformTargeting() {
        canPerformTargeting = false;

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
                canPerformTargeting = false;
                return;
            }
            canPerformTargeting = false;
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
                    canPerformTargeting = false;
                    return;
                }
                canPerformTargeting = false;
                super.execute();
                return;
            }
        } else {
            ticksWithoutTarget = 0;
        }

        positionAtFrame = swerveDrive.getRobotRelativePositionSince(camera.getLatestFrameWithTarget().getTimestampSeconds());
        if(positionAtFrame.cacheOverrun) {
            isFinished = true;
            canPerformTargeting = false;
            return;
        }

        canPerformTargeting = true;
    }

    protected void executeTargeting() {
        // If canPerformTargeting is false, don't perform targeting
        if(!canPerformTargeting) {
            return;
        }

        // Since getting here means canPerformTargeting is true, reset it to false and perform targeting
        canPerformTargeting = false;

        // --------- Calculate Speed ---------
        //double totalSpeed = Math.pow(Math.abs(calcX()), SPEED_SMOOTHING_MULTIPLIER) + Math.pow(Math.abs(calcY()), SPEED_SMOOTHING_MULTIPLIER);
        double totalSpeed = calcSpeed();

        // Limit the speed delta
        conditionedSpeed = Utl.clip(totalSpeed, lastConditionedSpeed - MAX_SPEED_DELTA, lastConditionedSpeed + MAX_SPEED_DELTA);

        // Slows the robot down as we go longer without a target. Hopefully allows the robot to "catch" the target again
        conditionedSpeed *= ((double) (resumeDrivingTickThreshold - ticksWithoutTarget) / (double) resumeDrivingTickThreshold);

        // Clip robot speed to stay below max speed
        conditionedSpeed = Utl.clip(conditionedSpeed, 0.0, MAX_SPEED);


        // ------- Calculate Rotation --------
        // find HEADING at specified field angle closest to current HEADING
        AngleD fieldHeading = navX.getHeadingInfo().getClosestHeading(HEADING.getDegrees());
        navX.setExpectedHeading(fieldHeading);
        conditionedRotate = new AngleD(navX.getHeadingInfo().expectedHeading).subtract(new AngleD(navX.getHeadingInfo().heading))
                .getRadians() * ROTATION_KP;


        // ------- Calculate Direction -------
        conditionedDirection.atan2(calcY(), calcX());

        // Add HEADING offset
        conditionedDirection.add(HEADING);


        // Update lasts
        lastConditionedDirection = conditionedDirection;
        lastConditionedSpeed = conditionedSpeed;
        lastConditionedRotate = conditionedRotate;

        // Check if the robot is in the zone
        if(Math.abs(camera.getXFromLastTarget() - positionAtFrame.forward - X_POSITION) < inZoneThreshold && Math.abs(camera.getYFromLastTarget() - positionAtFrame.strafe - Y_POSITION) < inZoneThreshold) {
            ticksInZoneCounter++;
            swerveDrive.swerveDrive(AngleD.ZERO, 0.0, conditionedRotate * 0.1);
            if(ticksInZoneCounter > TICKS_IN_ZONE) {
                isFinished = true;
            }
            return;
        }

        swerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
    }
}
