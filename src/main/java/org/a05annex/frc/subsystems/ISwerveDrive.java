package org.a05annex.frc.subsystems;

import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;

/**
 * Our interface specific to controlling the swerve drive. This is abstracted into an interface, so we can build
 * mock implementations of the drive subsystem for testing and so that we can build swerve drive commands
 */
@SuppressWarnings("unused")
public interface ISwerveDrive {

    /**
     * The drive modes that are used in the {@link #getDriveMode()}, {@link #toggleDriveMode()}, and
     * {@link #setDriveMode(DriveMode)}
     */
    enum DriveMode {
        /**
         * Robot movement is relative to the field/driver. Forward stick moves downfield, regardless of
         * robot orientation; right stick moves right across the field regardless of robot orientation.
         * Rotation behaviour is unaffected by drive mode.
         */
        FIELD_RELATIVE,
        /**
         * Robot movement is relative to the robot or forward-facing robot camera. Forward stick moves in
         * the direction the robot is headed (the direction the camera is facing), regardless of robot
         * orientation on the field; right stick moves the robot to its right (the right of the direction
         * of the camera) regardless of robot orientation. Rotation behaviour is unaffected by drive mode.
         */
        ROBOT_RELATIVE
    }

    /**
     * Set the swerve drive geometry and calibration constants. This must be called in your {@code Robot.robotInit()}
     * to describe the geometry and calibration of the swerve drive before any commands using this geometry
     * (like drive commands) are called. It should <i><b>NEVER</b></i> be called a second time.
     *
     * @param driveLength   (double) The length of the drive in meters.
     * @param driveWidth    (double) The width of the drive in meters.
     * @param rfCalibration (double) The reading of the right front spin encoder when the wheel is facing
     *                      directly forward.
     * @param rrCalibration (double) The reading of the right rear spin encoder when the wheel is facing
     *                      directly forward.
     * @param lfCalibration (double) The reading of the left front spin encoder when the wheel is facing
     *                      directly forward.
     * @param lrCalibration (double) The reading of the left rear spin encoder when the wheel is facing
     *                      directly forward.
     * @param maxSpeedCalibration (double) A calibration factor for the swerve module max m/sec to correct the
     *                            msx m/sec computed from all of the spec sheets and mox module motor RPM to
     *                            the empirically measured max m/sec.
     */
    void setDriveGeometry(double driveLength, double driveWidth,
                                 double rfCalibration, double rrCalibration,
                                 double lfCalibration, double lrCalibration, double maxSpeedCalibration);

    /**
     * Get the wheel center to center distance between the front wheels and the rear wheels - the length of
     * the swerve drive.
     *
     * @return the wheel center to center length of the swerve drive.
     */
    double getDriveLength();

    /**
     * Get the wheel center to center distance between the left wheels and the right wheels - the width of
     * the swerve drive.
     *
     * @return the wheel center to center width of the swerve drive.
     */
    double getDriveWidth();

    /**
     * Get the maximum speed of this drive in meters/sec. This is the maximum speed with no rotation.
     *
     * @return the maximum speed (meters/sec).
     */
    double getMaxMetersPerSec();

    /**
     * Get the maximum rotational speed of the robot in radians/sec. This is the maximum rotational speed
     * with no translation.
     *
     * @return the maximum rotational speed (radians/sec)
     */
    double getMaxRadiansPerSec();

    /**
     * Set the field position of the robot. This is typically called at the beginning of the autonomous
     * command as the command that is run should know where the robot has been placed on the field. It
     * could also be called during play if machine vision, sensors, or some other method is available
     * o locate the robot on the field.
     *
     * @param fieldX  (double) The X location of the robot on the field.
     * @param fieldY  (double) The Y location of the robot on the field.
     * @param heading (AngleD) The heading of the robot on the field.
     */
    void setFieldPosition(double fieldX, double fieldY, AngleD heading);

    /**
     * Run the swerve drive with the specified {@code  forward}, {@code strafe}, and {@code rotation} chassis
     * relative components.
     *
     * @param forward  Drive forward. From -1 (full backwards) to 1 (full forwards.
     * @param strafe   Strafe right. From -1 (full left)  to 1 (full right).
     * @param rotation Clockwise rotation. From -1 (full counter-clockwise) to 1 (full clockwise).
     */
    void swerveDriveComponents(double forward, double strafe, double rotation);

    /**
     * Prepare the swerve drive to run with the swerve drive with the specified {@code  forward}, {@code strafe},
     * and {@code rotation} chassis relative components. 'Prepare', in this context, means orient all the modules,
     * so they are ready to perform this command but set the module speeds to 0.0; In this way movement can start
     * smoothly without additional module reorientation. This method is used to initialize the robot before the
     * start of an autonomous path.
     *
     * @param forward  Drive forward. From -1 (full backwards) to 1 (full forwards.
     * @param strafe   Strafe right. From -1 (full left)  to 1 (full right).
     * @param rotation Clockwise rotation. From -1 (full counter-clockwise) to 1 (full clockwise).
     */
    void prepareForDriveComponents(double forward, double strafe, double rotation);

    /**
     * Swerve drive with a robot-relative direction, a speed and a rotation speed.
     *
     * @param direction (AngleConstantD) The robot chassis relative direction in radians from -PI to
     *                         PI where 0.0 is towards the front of the robot, and positive is clockwise.
     * @param speed            (double) Speed from 0.0 to 1.0.
     * @param rotation         (double) Clockwise rotation speed from -1.0 to 1.0.
     */
    void swerveDrive(AngleConstantD direction, double speed, double rotation);

    /**
     * Toggle between {@link DriveMode#FIELD_RELATIVE} and {@link DriveMode#ROBOT_RELATIVE} (robot camera
     * relative) driving.
     */
    void toggleDriveMode();

    /**
     * Get the swerve drive mode, either {@link DriveMode#FIELD_RELATIVE} or {@link DriveMode#ROBOT_RELATIVE}.
     *
     * @return {@link DriveMode#FIELD_RELATIVE} if the drive mode is field relative, {@link DriveMode#ROBOT_RELATIVE}
     * if the drive mode is driver (robot camera) relative.
     */
    DriveMode getDriveMode();

    /**
     * Set the drive mode.
     *
     * @param driveMode {@link DriveMode#FIELD_RELATIVE} to set the drive mode is field relative,
     *                  {@link DriveMode#ROBOT_RELATIVE} to set the drive mode to driver (robot
     *                  camera) relative.
     */
    void setDriveMode(DriveMode driveMode);
    /**
     * Rotate the chassis to the specified heading with no field translation. This controls the module using distance
     * (i.e. moving a specified number of ticks) rather than speed because this adjustment of heading is faster
     * and more reliable.
     *
     * @param targetHeading (AngleConstantD) The desired chassis heading on the field.
     */
    void setHeading(AngleConstantD targetHeading);
}
