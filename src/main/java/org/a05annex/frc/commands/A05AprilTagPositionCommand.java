package org.a05annex.frc.commands;

import edu.wpi.first.math.util.Units;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.RobotPosition;
import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.frc.subsystems.SpeedCachedSwerve;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;

import static org.a05annex.frc.A05Constants.aprilTagSetDictionary;

/**
 * Command for positioning the robot based on AprilTag and SpeedCache data.
 */
public class A05AprilTagPositionCommand extends A05DriveCommand {

    protected final SpeedCachedSwerve swerveDrive = SpeedCachedSwerve.getInstance();
    protected SpeedCachedSwerve.RobotRelativePosition positionAtFrame;
    protected RobotPosition robotPosition;
    protected final A05Constants.AprilTagSet tagSet;
    protected boolean canPerformTargeting = false;
    protected int ticksWithoutTarget = 0;
    protected final int resumeDrivingTickThreshold = 100;
    protected final double inZoneThreshold;
    protected final int TICKS_IN_ZONE = 10;
    protected int ticksInZoneCounter;
    protected final double MAX_SPEED_DELTA = 0.075, HEADING_ROTATION_KP = 0.9, TARGET_ROTATION_KP = 0.9;
    protected final double X_POSITION, Y_POSITION, MAX_SPEED = 1.0, SPEED_SMOOTHING_MULTIPLIER = 0.8;
    protected final AngleD HEADING;
    protected final double X_MAX, X_MIN, Y_MAX, Y_MIN;
    protected boolean isFinished = false;

    /**
     * Constructs a new A05AprilTagPositionCommand.
     *
     * @param xPosition The target x-coordinate position for the robot.
     * @param yPosition The target y-coordinate position for the robot.
     * @param tagSetKey The key for the AprilTag set to target.
     */
    protected A05AprilTagPositionCommand(double xPosition, double yPosition, String tagSetKey) {
        super(SpeedCachedSwerve.getInstance());
        this.tagSet = aprilTagSetDictionary.get(tagSetKey);
        this.X_POSITION = xPosition;
        this.Y_POSITION = -yPosition;
        this.HEADING = tagSet.heading();
        this.X_MIN = tagSet.X_MIN;
        this.X_MAX = tagSet.X_MAX;
        this.Y_MIN = tagSet.Y_MIN;
        this.Y_MAX = tagSet.Y_MAX;

        if(xPosition >= 1) {
            inZoneThreshold = Units.inchesToMeters(0.5 * xPosition);
        } else {
            inZoneThreshold = Units.inchesToMeters(0.5 * Math.pow(xPosition, 2.5));
        }
    }

    @Override
    public void initialize() {
        robotPosition = RobotPosition.getRobotPosition(tagSet);
        ticksWithoutTarget = robotPosition.isNew ? 0 : resumeDrivingTickThreshold;
        ticksInZoneCounter = 0;
        isFinished = false;
        canPerformTargeting = false;
    }

    @Override
    public void execute() {
        robotPosition = RobotPosition.getRobotPosition(tagSet);
        checkIfCanPerformTargeting();
        executeTargeting();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.swerveDrive(AngleConstantD.ZERO, 0.0, 0.0);
    }

    /**
     * Calculates the X speed between -1 and 1.
     *
     * @return x speed
     */
    protected double calcX() {
        double center = (X_MAX + X_MIN) / 2.0;
        double scale = (X_MAX - X_MIN) / 2.0;
        return Utl.clip((robotPosition.x - positionAtFrame.forward - center) / scale - (X_POSITION - center) / scale, -1.0, 1.0);
    }

    /**
     * Calculates the Y speed between -1 and 1.
     *
     * @return y speed
     */
    protected double calcY() {
        double center = (Y_MAX + Y_MIN) / 2.0;
        double scale = (Y_MAX - Y_MIN) / 2.0;
        return Utl.clip((robotPosition.y - positionAtFrame.strafe - center) / scale - (Y_POSITION - center) / scale, -1.0, 1.0);
    }

    /**
     * Checks if the current target ID is valid.
     *
     * @return True if valid, false otherwise.
     */
    protected boolean isValidTargetID() {
        return robotPosition.isValid;
    }

    /**
     * Checks for cache overrun in the robot's position data.
     *
     * @return True if there's a cache overrun, false otherwise.
     */
    protected boolean cacheOverrun() {
        return swerveDrive.getRobotRelativePositionSince(robotPosition.pipelineResult.getTimestampSeconds()).cacheOverrun;
    }

    /**
     * Determines whether the robot can perform targeting.
     */
    protected void checkIfCanPerformTargeting() {
        canPerformTargeting = false;

        if(!isValidTargetID()) {
            if(driveXbox == null) {
                isFinished = true;
                return;
            }
            super.execute();
            return;
        }

        if(!robotPosition.isNew) {
            ticksWithoutTarget++;
            if(ticksWithoutTarget > resumeDrivingTickThreshold) {
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

        if(cacheOverrun()) {
            isFinished = true;
            return;
        }

        canPerformTargeting = true;
    }

    /**
     * Calculates the current speed of the robot.
     *
     * @return The calculated speed.
     */
    protected double calcSpeed() {
        double speed = Math.pow(Math.sqrt(Math.pow(Math.abs(calcX()), 2) + Math.pow(Math.abs(calcY()), 2)), SPEED_SMOOTHING_MULTIPLIER);
        speed = Utl.clip(speed, lastConditionedSpeed - MAX_SPEED_DELTA, lastConditionedSpeed + MAX_SPEED_DELTA);
        speed *= ((double) (resumeDrivingTickThreshold - ticksWithoutTarget) / (double) resumeDrivingTickThreshold);
        return Utl.clip(speed, 0.0, MAX_SPEED);
    }

    /**
     * Calculates the direction in which the robot should move.
     *
     * @param direction The AngleD object representing the current direction.
     */
    protected void calcDirection(AngleD direction) {
        direction.atan2(calcY(), calcX());
        direction.add(HEADING);
    }

    /**
     * Calculates the rotation needed to align with the target.
     *
     * @return The calculated rotation.
     */
    protected double calcRotation() {
        if(tagSet.useTargetForHeading) {
            return calcRotationTargetHeading();
        } else {
            return calcRotationFieldHeading();
        }
    }

    /**
     * Calculates the rotation based on the field heading.
     *
     * @return The calculated field heading rotation.
     */
    protected double calcRotationFieldHeading() {
        AngleD fieldHeading = navX.getHeadingInfo().getClosestHeading(HEADING);
        navX.setExpectedHeading(fieldHeading);
        return new AngleD(navX.getHeadingInfo().expectedHeading).
                subtract(new AngleD(navX.getHeadingInfo().heading)).getRadians() * HEADING_ROTATION_KP;
    }

    /**
     * Calculates the rotation based on the target heading.
     *
     * @return The calculated target heading rotation.
     */
    protected double calcRotationTargetHeading() {
        if(!robotPosition.isNew) {
            return 0.0;
        }
        return PhotonCameraWrapper.filterForTarget(robotPosition.pipelineResult, tagSet).getYaw() / 35.0 * TARGET_ROTATION_KP;
    }

    /**
     * Checks if the robot is within the specified zone.
     *
     * @return True if the robot is in the zone, false otherwise.
     */
    protected boolean checkInZone() {
        return Utl.inTolerance(robotPosition.x - positionAtFrame.forward, X_POSITION, inZoneThreshold)
                && Utl.inTolerance(robotPosition.y - positionAtFrame.strafe, Y_POSITION, inZoneThreshold);
    }

    /**
     * Executes the targeting routine for the robot.
     */
    protected void executeTargeting() {
        if(!canPerformTargeting) {
            return;
        }

        canPerformTargeting = false;

        conditionedSpeed = calcSpeed();
        conditionedRotate = calcRotation();
        calcDirection(conditionedDirection);

        lastConditionedDirection = conditionedDirection;
        lastConditionedSpeed = conditionedSpeed;
        lastConditionedRotate = conditionedRotate;

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
