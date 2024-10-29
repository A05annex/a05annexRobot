package org.a05annex.frc.commands;

import org.a05annex.frc.A05Constants;
import org.a05annex.frc.InferredRobotPosition;
import org.a05annex.frc.NavX;
import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.frc.subsystems.SpeedCachedSwerve;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;

import java.util.function.DoubleFunction;

/**
 * Command that controls the robot to reach a specified target position using AprilTags for localization.
 */
public class A05TagTargetCommand extends A05DriveCommand {

    /** Singleton instance of the {@link SpeedCachedSwerve} for efficient swerve drive control. */
    protected final SpeedCachedSwerve swerveDrive = SpeedCachedSwerve.getInstance();

    /** Latest inferred robot position, used for updating the robot's position from camera data. */
    protected InferredRobotPosition inferredRobotPosition;

    /** The radius (in meters) where the robot begins to slow down as it approaches the target. */
    private static double REDUCED_SPEED_RADIUS = 2.0;

    /** The radius (in meters) within which the robot switches to precise position control. */
    private static double POSITION_CONTROL_RADIUS = 0.15;

    /**
     * Defines modes for adjusting robot speed based on its distance to the target.
     * These modes transition based on distance thresholds.
     */
    protected enum MODE {
        FULL_SPEED(distance -> distance >= REDUCED_SPEED_RADIUS),
        REDUCED_SPEED(distance -> distance < REDUCED_SPEED_RADIUS && distance >= POSITION_CONTROL_RADIUS),
        POSITION_CONTROL(distance -> distance < POSITION_CONTROL_RADIUS),
        WAITING_FOR_TARGET(distance -> false);

        /** Function that determines if this mode should be applied based on distance. */
        final DoubleFunction<Boolean> function;

        MODE(DoubleFunction<Boolean> function) {
            this.function = function;
        }
    }

    /** Stores the previous movement mode for comparison to detect mode changes. */
    protected MODE lastMode = null;

    /** Stores the current movement mode of the robot. */
    protected MODE currentMode = null;

    /** Specifies the AprilTag set that identifies the target for the robot. */
    protected final A05Constants.AprilTagSet tagSet;

    /** Boolean flag to determine if targeting can proceed based on validation checks. */
    protected boolean canTarget;

    /** Boolean flag to indicate if the command is complete. */
    protected boolean isFinished = false;

    /** Maximum allowable change in speed per control cycle (20 ms), for smoother acceleration. */
    protected final double MAX_SPEED_DELTA = 0.075;

    /** Proportional constant for field heading control. */
    protected final double HEADING_ROTATION_KP = 0.9;

    /** Proportional constant for target heading control. */
    protected final double TARGET_ROTATION_KP = 0.9;

    /** Target x-coordinate for the robot. */
    protected final double X_POSITION;

    /** Target y-coordinate for the robot. */
    protected final double Y_POSITION;

    /** Target heading angle for field-relative movement, defined by the AprilTag set. */
    protected final AngleD HEADING;

    /**
     * Constructs a new command targeting a specified position and AprilTag.
     *
     * @param X_POSITION Target x-coordinate for the robot.
     * @param Y_POSITION Target y-coordinate for the robot.
     * @param tagSetKey Key of the AprilTag set used to determine the target.
     */
    public A05TagTargetCommand(double X_POSITION, double Y_POSITION, String tagSetKey) {
        super(SpeedCachedSwerve.getInstance());
        tagSet = A05Constants.aprilTagSetDictionary.get(tagSetKey);
        this.X_POSITION = X_POSITION;
        this.Y_POSITION = Y_POSITION;
        this.HEADING = tagSet.heading();
        REDUCED_SPEED_RADIUS = tagSet.REDUCED_SPEED_RADIUS;
        POSITION_CONTROL_RADIUS = tagSet.POSITION_CONTROL_RADIUS;
    }

    /** Initializes the command, setting up the initial inferred position and targeting status. */
    @Override
    public void initialize() {
        inferredRobotPosition = InferredRobotPosition.getInferredRobotPosition(tagSet);
        currentMode = findMode();
        lastMode = null;
        canTarget = false;
        isFinished = false;
    }

    /** Executes the command, updating the inferred position and controlling robot movement based on targeting status. */
    @Override
    public void execute() {
        /*
        Positional control does not update the cache, so we risk a cache error if we continue requesting a new position
        after we stopped adding new ControlRequests
         */
        if(lastMode != MODE.POSITION_CONTROL) {
            inferredRobotPosition = InferredRobotPosition.getInferredRobotPosition(tagSet);
            // We don't need to do this unless we get new data, so it can stay in the if statement
            currentMode = findMode();
        }

        // Does all checks to verify that data is valid and good to target with.
        verifyOkToTarget();

        // Performs all actual drive calculations and calls methods that run the motors
        calculateAndDrive();

        canTarget = false;
        lastMode = currentMode;
    }

    /**
     * Checks if the command should finish, which is true when either the position control is done or flagged finished.
     *
     * @return True if the command should finish, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return isFinished || (lastMode == MODE.POSITION_CONTROL && swerveDrive.isAbsoluteTranslateDone());
    }

    /**
     * Validates if targeting is allowed based on the validity of the inferred position.
     * If the inferred position is invalid, targeting is disabled.
     */
    protected void verifyOkToTarget() {
        if (!inferredRobotPosition.isValid) {
            if (driveXbox == null) {
                isFinished = true;
                return;
            }
            currentMode = MODE.WAITING_FOR_TARGET;
            return;
        }

        canTarget = true;
    }

    /**
     * Calculates and applies driving commands based on the current mode and targeting data.
     * The robot will drive towards the target or wait depending on the mode.
     */
    protected void calculateAndDrive() {
        if (!canTarget) {
            return;
        }

        if (currentMode == MODE.WAITING_FOR_TARGET) {
            super.execute(); // Use joystick drive until target is detected
            lastMode = currentMode;
            return;
        }

        if (currentMode == MODE.POSITION_CONTROL && lastMode != MODE.POSITION_CONTROL) {
            swerveDrive.startAbsoluteSmartTranslate(xError(), yError(), 0.5, 5000.0);
            conditionedSpeed = 0.0;
            conditionedRotate = 0.0;
            conditionedDirection = new AngleD().atan2(yError(), xError());
        }

        if (currentMode == MODE.FULL_SPEED || currentMode == MODE.REDUCED_SPEED) {
            calcDirection(conditionedDirection);
            conditionedRotate = calcRotation();
            conditionedSpeed = currentMode == MODE.FULL_SPEED ? 1.0 : calcSpeed();
            swerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
        }

        lastConditionedRotate = conditionedRotate;
        lastConditionedSpeed = conditionedSpeed;
        lastConditionedDirection = conditionedDirection;
    }

    /** Stops the robot by setting all drive parameters to zero. */
    @Override
    public void end(boolean interrupted) {
        swerveDrive.swerveDrive(AngleConstantD.ZERO, 0.0, 0.0);
    }

    /**
     * Calculates the x-distance error to the target.
     *
     * @return The error in x-direction from current position to target position.
     */
    protected double xError() {
        return inferredRobotPosition.x - X_POSITION;
    }

    /**
     * Calculates the y-distance error to the target.
     *
     * @return The error in y-direction from current position to target position.
     */
    protected double yError() {
        return Y_POSITION - inferredRobotPosition.y;
    }

    /**
     * Sets the correct direction for the robot based on target position and field heading.
     *
     * @param direction The angle object to set to the correct driving direction.
     */
    protected void calcDirection(AngleD direction) {
        direction.atan2(yError(), xError());
        direction.add(HEADING);
    }

    /**
     * Determines the required rotation to achieve the desired heading based on either target or field heading.
     *
     * @return The calculated rotation in radians.
     */
    protected double calcRotation() {
        if (tagSet.useTargetForHeading) {
            return calcRotationTargetHeading();
        } else {
            return calcRotationFieldHeading();
        }
    }

    /**
     * Calculates the rotation required for a field-relative heading.
     *
     * @return The calculated rotation to align to the field heading.
     */
    protected double calcRotationFieldHeading() {
        NavX.HeadingInfo headingInfo = navX.getHeadingInfo();
        AngleD fieldHeading = headingInfo.getClosestHeading(HEADING);
        navX.setExpectedHeading(fieldHeading);
        return fieldHeading.subtract(new AngleD(headingInfo.heading)).getRadians() * HEADING_ROTATION_KP;
    }

    /**
     * Calculates the rotation required to align with a target.
     *
     * @return The calculated rotation to align to the target heading.
     */
    protected double calcRotationTargetHeading() {
        if (!inferredRobotPosition.isNew) {
            return 0.0;
        }
        return PhotonCameraWrapper.filterForTarget(inferredRobotPosition.pipelineResult, tagSet).getYaw() / 35.0 * TARGET_ROTATION_KP;
    }

    /**
     * Calculates the speed of the robot based on distance to the target and limits speed changes.
     *
     * @return The conditioned speed for the current mode.
     */
    protected double calcSpeed() {
        double speed = Utl.clip(distance() / REDUCED_SPEED_RADIUS, lastConditionedSpeed - MAX_SPEED_DELTA, lastConditionedSpeed + MAX_SPEED_DELTA);
        return Utl.clip(speed, 0.0, 1.0);
    }

    /**
     * Calculates the distance from the robot's current position to the target.
     *
     * @return The distance to the target.
     */
    protected double distance() {
        return Utl.length(xError(), yError());
    }

    /**
     * Determines the current mode of operation for the robot based on distance thresholds.
     *
     * @return The current mode based on distance to the target.
     */
    protected MODE findMode() {
        /*
        isValid is only false when inferredRobotPosition last saw the wrong target, or there is a cache overrun (the tag
        is so old that the cache moved past it)
         */
        if(!inferredRobotPosition.isValid) {
            return MODE.WAITING_FOR_TARGET;
        }

        for (MODE mode : MODE.values()) {
            if (mode.function.apply(distance())) {
                return mode;
            }
        }
        return MODE.WAITING_FOR_TARGET;
    }
}
