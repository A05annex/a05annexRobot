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


public class A05TagTargetCommand extends A05DriveCommand {
    /**
     * The {@link SpeedCachedSwerve} Singleton pointer. This is a local variable since the swerve drive is called many times
     */
    protected final SpeedCachedSwerve swerveDrive = SpeedCachedSwerve.getInstance();
    /**
     * Stores the latest {@link InferredRobotPosition} from the cameras
     */
    protected InferredRobotPosition inferredRobotPosition;

    private static double REDUCED_SPEED_RADIUS = 2.0;
    private static double POSITION_CONTROL_RADIUS = 0.15;

    protected enum MODE {
        FULL_SPEED(distance -> distance >= REDUCED_SPEED_RADIUS),
        REDUCED_SPEED(distance -> distance < REDUCED_SPEED_RADIUS && distance >= POSITION_CONTROL_RADIUS),
        POSITION_CONTROL(distance -> distance < POSITION_CONTROL_RADIUS),
        WAITING_FOR_TARGET(distance -> false); // WAITING_FOR_TARGET needs to be last so that findCurrentPhase() works

        final DoubleFunction<Boolean> function;

        MODE(DoubleFunction<Boolean> function) {
            this.function = function;
        }
    }

    protected MODE lastMode = null;
    protected MODE currentMode = null;
    /**
     * The {@link A05Constants.AprilTagSet} that stores defines which tag to target with
     */
    protected final A05Constants.AprilTagSet tagSet;
    /**
     * Boolean flag used to specify if targeting can be performed. Primarily to enable child classes to call the
     * {@link #verifyOkToTarget()} if they don't need to modify the safeguard checks and only need to edit the
     * targeting algorithm.
     */
    protected boolean canTarget;
    /**
     * Boolean flag to make true when the command is finished.
     */
    protected boolean isFinished = false;
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
    /**
     * The heading to target for if the {@link A05Constants.AprilTagSet} says to use field relative headings.
     */
    protected final AngleD HEADING;
    /**
     * Constructs a new A05AprilTagPositionCommand.
     *
     * @param X_POSITION The target x-coordinate position for the robot.
     * @param Y_POSITION The target y-coordinate position for the robot.
     * @param tagSetKey The key for the AprilTag set to target.
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

    @Override
    public void initialize() {
        inferredRobotPosition = InferredRobotPosition.getInferredRobotPosition(tagSet);
        currentMode = findMode();
        lastMode = null; // Set to null since we haven't complete a cycle yet
        canTarget = false;
        isFinished = false;
    }

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

        calculateAndDrive();

        canTarget = false;
        lastMode = currentMode;
    }


    @Override
    public boolean isFinished() {
        return isFinished || (lastMode == MODE.POSITION_CONTROL && swerveDrive.isAbsoluteTranslateDone());
    }

    protected void verifyOkToTarget() {
        if(!inferredRobotPosition.isValid) {
            if(driveXbox == null) {
                isFinished = true;
                return;
            }
            currentMode = MODE.WAITING_FOR_TARGET;
            return;
        }

        canTarget = true;
    }

    protected void calculateAndDrive() {
        if(!canTarget) {
            return;
        }

        if(currentMode == MODE.WAITING_FOR_TARGET) {
            super.execute(); // Run joystick drive until we find a target
            lastMode = currentMode;
            return;
        }

        if(currentMode == MODE.POSITION_CONTROL && lastMode != MODE.POSITION_CONTROL) { // We only want to do this once
            swerveDrive.startAbsoluteSmartTranslate(xError(), yError(), 0.5, 5000.0);
            conditionedSpeed = 0.0;
            conditionedRotate = 0.0;
            conditionedDirection = new AngleD().atan2(yError(), xError());
        }

        if(currentMode == MODE.FULL_SPEED || currentMode == MODE.REDUCED_SPEED) {
            calcDirection(conditionedDirection);
            conditionedRotate = calcRotation();
            conditionedSpeed = currentMode == MODE.FULL_SPEED ? 1.0 : calcSpeed();
            swerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
        }

        lastConditionedRotate = conditionedRotate;
        lastConditionedSpeed = conditionedSpeed;
        lastConditionedDirection = conditionedDirection;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.swerveDrive(AngleConstantD.ZERO, 0.0, 0.0);
    }

    protected double xError() {
        return inferredRobotPosition.x - X_POSITION;
    }

    protected double yError() {
        return Y_POSITION - inferredRobotPosition.y;
    }

    /**
     * Calculates the direction in which the robot should drive.
     *
     * @param direction The AngleD object you want to set to the correct direction.
     */
    protected void calcDirection(AngleD direction) {
        direction.atan2(yError(), xError());
        direction.add(HEADING);
    }

    /**
     * Finds the rotation needed to achieve a desired heading by choosing the correct calculation method
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
        NavX.HeadingInfo headingInfo = navX.getHeadingInfo();
        AngleD fieldHeading = headingInfo.getClosestHeading(HEADING);
        navX.setExpectedHeading(fieldHeading);
        return fieldHeading.subtract(new AngleD(headingInfo.heading)).getRadians() * HEADING_ROTATION_KP;
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
     * Calculates the current speed of the robot.
     *
     * @return The calculated speed.
     */
    protected double calcSpeed() {
        double speed = Utl.clip(distance() / REDUCED_SPEED_RADIUS, lastConditionedSpeed - MAX_SPEED_DELTA, lastConditionedSpeed + MAX_SPEED_DELTA);
        return Utl.clip(speed, 0.0, 1.0);
    }

    protected double distance() {
        return Utl.length(xError(), yError());
    }

    protected MODE findMode() {
        /*
        isValid is only false when inferredRobotPosition last saw the wrong target, or there is a cache overrun (the tag
        is so old that the cache moved past it)
         */
        if(!inferredRobotPosition.isValid) {
            return MODE.WAITING_FOR_TARGET;
        }

        for(MODE mode : MODE.values()) {
            if(mode.function.apply(distance())) {
                return mode;
            }
        }
        return MODE.WAITING_FOR_TARGET;
    }
}
