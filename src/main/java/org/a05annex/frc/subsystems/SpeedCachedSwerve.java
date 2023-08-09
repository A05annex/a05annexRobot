package org.a05annex.frc.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.a05annex.frc.NavX;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;
import org.jetbrains.annotations.NotNull;

/**
 * This is a layer that goes on top of the swerve drive to provide caching of past drive commands for some
 * period of time so that the relative position of the robot can be approximated for some time in the past. The
 * scenario for use is that there is a sensor system, such as vision processing of a target, that estimates
 * a robot's position relative to the target; however, it has a several command cycle latency. This means
 * that a control algorithm based on the robot's position relative to the target is using information about that
 * position stale by several cycles when used - and needs to be corrected for the robot's current position.
 * <p>
 * This cache provides the data and processing to predict where the robot is now, relative to the location,
 * location reported for some time in the past.
 */
public class SpeedCachedSwerve implements ISwerveDrive {

    /**
     * The single instance of this class
     */
    private final static SpeedCachedSwerve INSTANCE = new SpeedCachedSwerve();

    /**
     * Get the single instance of this class.
     *
     * @return the single instance of this class.
     */
    public static SpeedCachedSwerve getInstance() {
        return INSTANCE;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // This is the cache of the last requests to the swerve drive and the processing. Here we cache a collection
    // of timestamped robot move commands
    // -----------------------------------------------------------------------------------------------------------------

    /**
     *
     */
    public static class ControlRequest {
        double forward;
        double strafe;
        double rotation;
        double timeStamp;

        void set(double forward, double strafe, double rotation, double timeStamp) {
            this.forward = forward;
            this.strafe = strafe;
            this.rotation = rotation;
            this.timeStamp = timeStamp;
        }

        public double getForward() {
            return forward;
        }

        public double getStrafe() {
            return forward;
        }

        public double getRotation() {
            return forward;
        }

        public double getTimeStamp() {
            return timeStamp;
        }
    }

    public static class RobotRelativePosition {
        public final double forward;
        public final double strafe;
        public final AngleD heading;
        public final double timeStamp;
        public final boolean cacheOverrun;

        RobotRelativePosition(double forward, double strafe, AngleD heading,
                              double timeStamp, boolean cacheOverrun) {
            this.forward = forward;
            this.strafe = strafe;
            this.heading = heading;
            this.timeStamp = timeStamp;
            this.cacheOverrun = cacheOverrun;
        }
    }

    ControlRequest[] controlRequests = null;
    int cacheLength;
    int mostRecentControlRequest;

    private DriveSubsystem driveSubsystem = null;
    private DriveMode driveMode = DriveMode.FIELD_RELATIVE;
    private double driveLength = 0.0;
    private double driveWidth = 0.0;
    private double maxMetersPerSec = 3.136;
    private double maxRadiansPerSec = 3.136;

    private SpeedCachedSwerve() {
        // the constructor does nothing ...
    }

    public ControlRequest getMostRecentControlRequest() {
        return controlRequests[mostRecentControlRequest];
    }

    /**
     * Get the robot position now, relative to where the robot was at the specified time. For example if the robot
     * were approaching a target, and the targeting software reported that the robot was 2.0m from the target and
     * 0.5m left of the target. Assume we want the robot to stay on its current heading and
     * the robot has been moving 1.0m/s forward, and 0.5m/s to its right with a rotation of 0.0rad/s. If the
     * camera latency is 100ms or 0.1 sec, the returned position will report the robot has moved forward 0.1m
     * with a strafe of 0.05m, so the actual robot position is currently 1.9m from the target and 0.45m to the
     * left of the target.
     * <p>
     * NOTE: If the cache does not store enough data to accumulate the motion back tom {@code sinceTime}, then the
     * motion is the furthest back that can be estimated, and {@link RobotRelativePosition#cacheOverrun} will
     * be set to {@code true} in the returned position.
     *
     * @param sinceTime The time from which you want to know the robot's new position. In the context of the
     *                  example, this is the FPGA timestamp (in seconds) reported by the targeting software at
     *                  which the targeting data was valid.
     * @return The estimated robot position at {@code sinceTime}
     */
    @NotNull
    public RobotRelativePosition getRobotRelativePositionSince(double sinceTime) {
        return getRobotRelativePositionSince(Timer.getFPGATimestamp(), sinceTime);
    }

    /**
     * See {@link #getRobotRelativePositionSince(double)} for documentation. This method exists for
     * testing scenarios where the start time is not <i>now</i>, but is well known for the purpose of testing.
     *
     * @param currentTime The <i>current time</i>, which is the FPGA timestamp (in seconds) during match play,
     *                    but will be assigned as required for testing.
     * @param sinceTime   The time (FPGA timestamp in seconds) from which you want to know the robot's new position.
     * @return The estimated robot position at {@code sinceTime}
     */
    @NotNull
    RobotRelativePosition getRobotRelativePositionSince(double currentTime, double sinceTime) {
        int backIndex = mostRecentControlRequest;
        double forward = 0.0;
        double strafe = 0.0;
        double headingRadians = 0.0;
        boolean cacheOverrun = false;
        while (controlRequests[backIndex].timeStamp > sinceTime) {
            if (currentTime > controlRequests[backIndex].timeStamp) {
                double deltaTime = currentTime - controlRequests[backIndex].timeStamp;
                forward += deltaTime * controlRequests[backIndex].forward * maxMetersPerSec;
                strafe += deltaTime * controlRequests[backIndex].strafe * maxMetersPerSec;
                headingRadians += deltaTime * controlRequests[backIndex].rotation * maxRadiansPerSec;
                currentTime = controlRequests[backIndex].timeStamp;
            }
            backIndex--;
            if (backIndex < 0) {
                backIndex = cacheLength - 1;
            }
            if (backIndex == mostRecentControlRequest) {
                // There are not enough entries in the cache
                cacheOverrun = true;
                break;
            }
        }
        return new RobotRelativePosition(forward, strafe,
                new AngleD(AngleUnit.RADIANS, headingRadians), sinceTime, cacheOverrun);
    }

    /**
     * Adds a control request to the cache. This is a package function where the timestamp is specified, so the
     * cache can be loaded for test scenarios.
     *
     * @param forward   Forward speed (-1.0 to 1.0)
     * @param strafe    Strafe speed (-1.0 to 1.0)
     * @param rotation  Rotation speed (-1.0 to 1.0)
     * @param timestamp The <i>current time</i>, which is the FPGA timestamp (in seconds) during match play,
     *                  but will be assigned as required for testing.
     */
    void addControlRequest(double forward, double strafe, double rotation, double timestamp) {
        // increment the index to the array of cached control requests
        mostRecentControlRequest++;
        if (mostRecentControlRequest >= cacheLength) {
            mostRecentControlRequest = 0;
        }
        // save this request with a timestamp
        controlRequests[mostRecentControlRequest].set(forward, strafe, rotation, timestamp);
    }

    public void setCacheLength(int cacheLength) {
        this.cacheLength = cacheLength;
        mostRecentControlRequest = -1;
        controlRequests = new ControlRequest[cacheLength];

        for (int i = 0; i < cacheLength; i++) {
            controlRequests[i] = new ControlRequest();
        }
    }

    public int getCacheLength() {
        return cacheLength;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // The wrapping for the underlying swerve drive subsystem
    // -----------------------------------------------------------------------------------------------------------------
    public void setDriveSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        if (driveSubsystem != null) {
            driveMode = driveSubsystem.getDriveMode();
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // ISwerveDrive implementation. Most of this is pass-through to the wrapped swerve drive
    // -----------------------------------------------------------------------------------------------------------------
    @Override
    public void setDriveGeometry(double driveLength, double driveWidth,
                                 double rfCalibration, double rrCalibration,
                                 double lfCalibration, double lrCalibration, double maxSpeedCalibration) {
        if (driveSubsystem != null) {
            // There is a drive subsystem - so we get everything we need after the geometry is set.
            driveSubsystem.setDriveGeometry(driveLength, driveWidth,
                    rfCalibration, rrCalibration, lfCalibration, lrCalibration, maxSpeedCalibration);
            this.driveLength = driveSubsystem.getDriveLength();
            this.driveWidth = driveSubsystem.getDriveWidth();
            maxMetersPerSec = driveSubsystem.getMaxMetersPerSec();
            maxRadiansPerSec = driveSubsystem.getMaxRadiansPerSec();
        } else {
            // We are testing, no drive subsystem, so use the geometry to duplicate what would happen
            // in the Drive subsystem
            this.driveLength = driveLength;
            this.driveWidth = driveWidth;
            maxMetersPerSec = Mk4NeoModule.MAX_METERS_PER_SEC * maxSpeedCalibration;
            double driveDiagonal = Utl.length(this.driveLength, this.driveWidth);
            maxRadiansPerSec = maxMetersPerSec / (0.5 * driveDiagonal);
        }
    }

    @Override
    public void calibrate() {
        if (driveSubsystem != null) {
            driveSubsystem.calibrate();
        }
    }

    @Override
    public Subsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    @Override
    public double getDriveLength() {
        if (driveSubsystem != null) {
            return driveSubsystem.getDriveLength();
        }
        return driveLength;
    }

    @Override
    public double getDriveWidth() {
        if (driveSubsystem != null) {
            return driveSubsystem.getDriveWidth();
        }
        return driveWidth;
    }

    @Override
    public double getMaxMetersPerSec() {
        if (driveSubsystem != null) {
            return driveSubsystem.getMaxMetersPerSec();
        }
        return maxMetersPerSec;
    }

    @Override
    public double getMaxRadiansPerSec() {
        if (driveSubsystem != null) {
            return driveSubsystem.getMaxRadiansPerSec();
        }
        return maxRadiansPerSec;
    }

    @Override
    public void setFieldPosition(double fieldX, double fieldY, AngleD heading) {
        if (driveSubsystem != null) {
            driveSubsystem.setFieldPosition(fieldX, fieldY, heading);
        }
    }

    @Override
    public void swerveDriveComponents(double forward, double strafe, double rotation) {
        // send the request to the drive if we have one
        if (driveSubsystem != null) {
            driveSubsystem.swerveDriveComponents(forward, strafe, rotation);
        }
        addControlRequest(forward, strafe, rotation, Timer.getFPGATimestamp());
    }

    @Override
    public void prepareForDriveComponents(double forward, double strafe, double rotation) {
        if (driveSubsystem != null) {
            driveSubsystem.prepareForDriveComponents(forward, strafe, rotation);
        }
    }

    @Override
    public void swerveDrive(AngleConstantD direction, double speed, double rotation) {
        if (driveMode == DriveMode.FIELD_RELATIVE) {
            AngleD chassisDirection = new AngleD(direction).subtract(NavX.getInstance().getHeading());
            swerveDriveComponents(chassisDirection.cos() * speed,
                    chassisDirection.sin() * speed, rotation);
        } else {
            swerveDriveComponents(direction.cos() * speed,
                    direction.sin() * speed, rotation);
        }
    }

    @Override
    public void toggleDriveMode() {
        if (driveSubsystem != null) {
            driveSubsystem.toggleDriveMode();
            driveMode = driveSubsystem.getDriveMode();
        } else {
            driveMode = (driveMode == DriveMode.FIELD_RELATIVE) ?
                    DriveMode.ROBOT_RELATIVE : DriveMode.FIELD_RELATIVE;
        }
    }

    @Override
    public DriveMode getDriveMode() {
        if (driveSubsystem != null) {
            return driveSubsystem.getDriveMode();
        }

        return driveMode;
    }

    @Override
    public void setDriveMode(DriveMode driveMode) {
        if (driveSubsystem != null) {
            driveSubsystem.setDriveMode(driveMode);
        }
        this.driveMode = driveMode;
    }

    @Override
    public void setHeading(AngleConstantD targetHeading) {
        if (driveSubsystem != null) {
            driveSubsystem.setHeading(targetHeading);
        }
    }

    @Override
    public void translate(double distanceForward, double distanceStrafe) {
        if (driveSubsystem != null) {
            driveSubsystem.translate(distanceForward, distanceStrafe);
        }
    }

    @Override
    public void startAbsoluteTranslate(double distanceForward, double distanceStrafe, double maxSpeed) {
        if (driveSubsystem != null) {
            driveSubsystem.startAbsoluteTranslate(distanceForward, distanceStrafe, maxSpeed);
        }
    }

    @Override
    public void startAbsoluteSmartTranslate(double distanceForward, double distanceStrafe,
                                            double maxSpeed, double maxAcceleration) {
        if (driveSubsystem != null) {
            driveSubsystem.startAbsoluteSmartTranslate(distanceForward, distanceStrafe, maxSpeed, maxAcceleration);
        }
    }

    @Override
    public boolean isAbsoluteTranslateDone() {
        if (driveSubsystem != null) {
            return driveSubsystem.isAbsoluteTranslateDone();
        }
        return false;
    }
}

