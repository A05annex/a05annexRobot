package org.a05annex.frc.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.frc.subsystems.SpeedCachedSwerve;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;

import static org.a05annex.frc.A05Constants.aprilTagSetDictionary;


public class A05AprilTagPositionCommand extends A05DriveCommand {

    protected final SpeedCachedSwerve swerveDrive = SpeedCachedSwerve.getInstance();

    protected SpeedCachedSwerve.RobotRelativePosition positionAtFrame;

    protected final A05Constants.AprilTagSet tagSet;

    protected final PhotonCameraWrapper camera;

    protected boolean canPerformTargeting = false;

    // Counter for how many ticks the robot hasn't seen a valid target
    protected int ticksWithoutTarget = 0;

    // Maximum number of ticks where the robot hasn't had a valid target before resuming normal driving
    protected final int resumeDrivingTickThreshold = 50;

    // The size of the "box" in which the robot is considered in the right position
    protected final double inZoneThreshold;

    // Number of ticks where the robot needs to be in the zone to end the command
    protected final int TICKS_IN_ZONE = 10;

    // Counter for how many ticks the robot has been in the zone
    protected int ticksInZoneCounter;

    // Drive and control constants
    protected final double MAX_SPEED_DELTA = 0.075, HEADING_ROTATION_KP = 0.9, TARGET_ROTATION_KP = 0.9;
    protected final double X_POSITION, Y_POSITION, MAX_SPEED = 1.0, SPEED_SMOOTHING_MULTIPLIER = 0.8;
    protected final int[] aprilTagIds;
    protected final AngleD HEADING;
    protected final double X_MAX, X_MIN, Y_MAX, Y_MIN;

    protected boolean isFinished = false;

    protected A05AprilTagPositionCommand(PhotonCameraWrapper camera,
                                      double xPosition, double yPosition, String tagSetKey) {
        // NOTE: the super adds the drive subsystem requirement
        super(SpeedCachedSwerve.getInstance());

        this.camera = camera;

        this.tagSet = aprilTagSetDictionary.get(tagSetKey);

        this.X_POSITION = xPosition;
        this.Y_POSITION = -yPosition;
        this.HEADING = tagSet.heading;
        this.aprilTagIds = tagSet.tagIDs;
        this.X_MIN = tagSet.X_MIN;
        this.X_MAX = tagSet.X_MAX;
        this.Y_MIN = tagSet.Y_MIN;
        this.Y_MAX = tagSet.Y_MAX;

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
        camera.updateTrackingData();
        /*
          Is there a good target?
          Yes: set ticksWithoutTarget to 0
          No: set ticksWithoutTarget to the resumeDrivingTickThreshold because that is the point at which we resume normal driver control

          If there is not immediately a target, the driver can keep going until there is a target, which means the
          robot won't randomly stop meaning we move faster and smoother
        */
        ticksWithoutTarget = camera.isTargetDataNew(tagSet) ? 0 : resumeDrivingTickThreshold;

        // Reset values for the start of a new command
        ticksInZoneCounter = 0;
        isFinished = false;
        canPerformTargeting = false;
    }

    @Override
    public void execute() {
        camera.updateTrackingData();

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

    /**
     * Calculates the X speed between -1 and 1
     *
     * @return x speed
     */
    protected double calcX() {
        double center = (X_MAX + X_MIN) / 2.0;
        double scale = (X_MAX - X_MIN) / 2.0;
        return Utl.clip((camera.getXFromLastTarget(tagSet) - positionAtFrame.forward - center) / scale - (X_POSITION - center) / scale, -1.0, 1.0);
    }

    /**
     * Calculates the Y speed between -1 and 1
     *
     * @return y speed
     */
    protected double calcY() {
        double center = (Y_MAX + Y_MIN) / 2.0;
        double scale = (Y_MAX - Y_MIN) / 2.0;
        return Utl.clip((camera.getYFromLastTarget(tagSet) - positionAtFrame.strafe - center) / scale - (Y_POSITION - center) / scale, -1.0, 1.0);
    }

    protected boolean isValidTargetID() {;
        return camera.getTarget(tagSet) != null;
    }

    protected boolean cacheOverrun() {
        return swerveDrive.getRobotRelativePositionSince(camera.getLatestTargetTime()).cacheOverrun;
    }

    protected void checkIfCanPerformTargeting() {
        canPerformTargeting = false;

        if(!isValidTargetID()) {
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
        if(!camera.isTargetDataNew(tagSet)) {
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

        if(cacheOverrun()) {
            isFinished = true;
            canPerformTargeting = false;
            return;
        }

        canPerformTargeting = true;
    }

    protected double calcSpeed() {
        double speed = Math.pow(Math.sqrt(Math.pow(Math.abs(calcX()), 2) + Math.pow(Math.abs(calcY()), 2)), SPEED_SMOOTHING_MULTIPLIER);

        // Limit the speed delta
        speed = Utl.clip(speed, lastConditionedSpeed - MAX_SPEED_DELTA, lastConditionedSpeed + MAX_SPEED_DELTA);

        // Slows the robot down as we go longer without a target. Hopefully allows the robot to "catch" the target again
        speed *= ((double) (resumeDrivingTickThreshold - ticksWithoutTarget) / (double) resumeDrivingTickThreshold);

        // Clip robot speed to stay below max speed
        return Utl.clip(speed, 0.0, MAX_SPEED);
    }

    protected void calcDirection(AngleD direction) {

        direction.atan2(calcY(), calcX());

        // Add HEADING offset
        direction.add(HEADING);
    }

    protected double calcRotation() {
        if(tagSet.useTargetForHeading) {
            return calcRotationTargetHeading();
        }
        else {
            return calcRotationFieldHeading();
        }
    }

    private double calcRotationFieldHeading() {
        AngleD fieldHeading = navX.getHeadingInfo().getClosestHeading(HEADING);
        navX.setExpectedHeading(fieldHeading);
        return new AngleD(navX.getHeadingInfo().expectedHeading).
                subtract(new AngleD(navX.getHeadingInfo().heading)).getRadians() * HEADING_ROTATION_KP;
    }

    private double calcRotationTargetHeading() {
        if(!camera.isTargetDataNew(tagSet)) {
            return 0.0;
        }

        return camera.getTarget(tagSet).getYaw() / 100.0 * TARGET_ROTATION_KP;
    }

    protected boolean checkInZone() {
        return Utl.inTolerance(camera.getXFromLastTarget(tagSet) - positionAtFrame.forward, X_POSITION, inZoneThreshold)
                && Utl.inTolerance(camera.getYFromLastTarget(tagSet) - positionAtFrame.strafe, Y_POSITION, inZoneThreshold);
    }

    protected void executeTargeting() {
        // If canPerformTargeting is false, don't perform targeting
        if(!canPerformTargeting) {
            return;
        }

        // Since getting here means canPerformTargeting was true, reset it to false and perform targeting
        canPerformTargeting = false;

        // --------- Calculate Speed ---------
        conditionedSpeed = calcSpeed();


        // ------- Calculate Rotation --------
        calcRotation();


        // ------- Calculate Direction -------
        calcDirection(conditionedDirection);


        // Update lasts
        lastConditionedDirection = conditionedDirection;
        lastConditionedSpeed = conditionedSpeed;
        lastConditionedRotate = conditionedRotate;

        // Check if the robot is in the zone
        if(checkInZone()) {
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
