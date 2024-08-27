package org.a05annex.frc.commands;

import edu.wpi.first.math.util.Units;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.InferredRobotPosition;
import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.frc.subsystems.SpeedCachedSwerve;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;

import static org.a05annex.frc.A05Constants.aprilTagSetDictionary;

/**
 * Command for positioning the robot based on InferredRobotPosition which accounts for both AprilTag and SpeedCache position
 */
public class A05AprilTagPositionCommand extends A05DriveCommand {
    /**
     * The {@link SpeedCachedSwerve} Singleton pointer. This is a local variable since the swerve drive is called many times
     */
    protected final SpeedCachedSwerve swerveDrive = SpeedCachedSwerve.getInstance();
    /**
     * Stores the latest {@link InferredRobotPosition} from the cameras
     */
    protected InferredRobotPosition inferredRobotPosition;
    /**
     * The {@link A05Constants.AprilTagSet} that stores defines which tag to target with
     */
    protected final A05Constants.AprilTagSet tagSet;
    /**
     * Boolean flag used to specify if targeting can be performed. Primarily to enable child classes to call the
     * {@link #checkIfCanPerformTargeting()} if they don't need to modify the safeguard checks and only need to edit the
     * targeting algorithm.
     */
    protected boolean canPerformTargeting = false;
    //TODO: Talk with roy about whether we still want/need this function or if with the cache we should edit or remove
    // how it works
    /**
     * Counter to track the amount of 20ms control cycles the robot has gone without seeing a new target. Used as a
     * multiplier on the {@link #conditionedSpeed} to slow the robot down as it goes longer without a target
     */
    protected int ticksWithoutTarget = 0;
    /**
     * Maximum amount of 20ms control cycles that the robot can go before it resumes joystick driving.
     */
    protected final int resumeDrivingTickThreshold = 100;
    //TODO: Revisit this functionality. it may need to be edited with the implementation of InferredRobotPositions.
    /**
     * Distance in meters that defines the maximum deviation from the target position where the robot is still
     * considered to have arrived at the target. This is needed since there is minor variance in the PhotonVision's
     * reported position and our inability to drive to an exact location. Set by a function relative to the target X in
     * the constructor
     */
    protected final double inZoneThreshold;
    /**
     * Required number of consecutive control cycles where the robot's position is within the {@link #inZoneThreshold} of
     * the target position to be considered to have arrived at the target.
     */
    protected final int TICKS_IN_ZONE = 10;
    /**
     * Counts the number of consecutive control cycles (ticks) where the robot was inside the {@link #inZoneThreshold}.
     */
    protected int ticksInZoneCounter;
    /**
     * The maximum change in speed (0.0 to 1.0) per 20ms control cycle. This prevents harsh accelerations or
     * decelerations that could excessively damage treads or cause burnouts.
     */
    protected final double MAX_SPEED_DELTA = 0.075;
    /**
     * The kP to use when controlling the robot heading with the goal of maintaining a field heading.
     */
    protected final double HEADING_ROTATION_KP = 0.9;
    /**
     * The kP to use when controlling the robot heading with the goal of facing a target.
     */
    protected final double TARGET_ROTATION_KP = 0.9;
    /**
     * The X position to target for. Passed in as a parameter.
     */
    protected final double X_POSITION;
    /**
     * The Y position to target for. Pass in as a parameter.
     */
    protected final double Y_POSITION;
    //TODO: Revisit this. Is it still needed now that we have the cache, non-rolling-shutter cameras, and better ways to handle losing the target for
    // a frame or two
    /**
     * The maximum speed that the robot will be clipped under. If the robot has trouble maintaining a target lock, this
     * may need to be reduced.
     */
    protected final double MAX_SPEED = 1.0;
    /**
     * This is an exponent applied to the speed (0.0 to 1.0) which changes how the robot responds. A value above 1.0
     * results in slower overall targeting, a value greater than 1.0 results in a more aggressive targeting speed.
     */
    protected final double SPEED_SMOOTHING_MULTIPLIER = 0.8;
    /**
     * The heading to target for if the {@link A05Constants.AprilTagSet} says to use field relative headings.
     */
    protected final AngleD HEADING;
    protected final double X_MAX, X_MIN, Y_MAX, Y_MIN;
    /**
     * Boolean flag to make true when the command is finished.
     */
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
        inferredRobotPosition = InferredRobotPosition.getInferredRobotPosition(tagSet);
        ticksWithoutTarget = inferredRobotPosition.isNew ? 0 : resumeDrivingTickThreshold;
        ticksInZoneCounter = 0;
        isFinished = false;
        canPerformTargeting = false;
    }

    @Override
    public void execute() {
        inferredRobotPosition = InferredRobotPosition.getInferredRobotPosition(tagSet);
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
        return Utl.clip((inferredRobotPosition.x - center) / scale - (X_POSITION - center) / scale, -1.0, 1.0);
    }

    /**
     * Calculates the Y speed between -1 and 1.
     *
     * @return y speed
     */
    protected double calcY() {
        double center = (Y_MAX + Y_MIN) / 2.0;
        double scale = (Y_MAX - Y_MIN) / 2.0;
        return Utl.clip((inferredRobotPosition.y - center) / scale - (Y_POSITION - center) / scale, -1.0, 1.0);
    }

    /**
     * Checks if the current target ID is valid.
     *
     * @return True if valid, false otherwise.
     */
    protected boolean isRobotPositionValid() {
        return inferredRobotPosition.isValid;
    }

    /**
     * Determines whether the robot can perform targeting.
     * <p>
     * Sets the boolean flag {@link #canPerformTargeting} enabling a wrapper to call the can perform targeting and pull the
     * result
     * <p>
     * If {@link #canPerformTargeting} will be false, calls {@code super.execute()} which is
     * {@link A05DriveCommand#execute()} and thus executes a joystick drive as if the
     * {@link org.a05annex.frc.subsystems.DriveSubsystem} is running its default command
     */
    protected void checkIfCanPerformTargeting() {
        canPerformTargeting = false;

        if(!isRobotPositionValid()) {
            if(driveXbox == null) {
                isFinished = true;
                return;
            }
            super.execute();
            return;
        }

        if(!inferredRobotPosition.isNew) {
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
        if(!inferredRobotPosition.isNew) {
            return 0.0;
        }
        return PhotonCameraWrapper.filterForTarget(inferredRobotPosition.pipelineResult, tagSet).getYaw() / 35.0 * TARGET_ROTATION_KP;
    }

    /**
     * Checks if the robot is within the specified zone.
     *
     * @return True if the robot is in the zone, false otherwise.
     */
    protected boolean checkInZone() {
        return Utl.inTolerance(inferredRobotPosition.x, X_POSITION, inZoneThreshold)
                && Utl.inTolerance(inferredRobotPosition.y, Y_POSITION, inZoneThreshold);
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
