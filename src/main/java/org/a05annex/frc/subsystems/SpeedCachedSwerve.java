package org.a05annex.frc.subsystems;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.a05annex.frc.NavX;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * This is a layer that goes on top of the swerve drive to provide caching of past drive commands for some
 * period of time so that the motion from a previously known field position at a previous time can be projected
 * to a field position at the current time. The use case is with vision systems tha take some time (typically
 * 20 to 120ms) to analyze the field targets and report their position relative the the target, and it follows
 * that if we know the field position of the target we can compute the field position of the robot. Unfortunately,
 * that position is 20-120ms old, so running a PID loop to put the robot at a specific point on the field requires
 * very low gains to prevent oscillation around the target position. In addition to the processing latency issue,
 * there is also the case where when the robot is in the desired field position, it cannot see the target, so it
 * may be necessary to predict the current position based on the last time the target was visible (perhaps
 * one second ago).
 * <p>
 * For each command cycle we save:
 * <ul>
 *     <li>forward speed (-1.0 to +1.0);</li>
 *     <li>strafe speed (-1.0 to +1.0);</li>
 *     <li>rotation speed (-1.0 to +1.0);</li>
 *     <li>expected heading;</li>
 *     <li>actual heading.</li>
 * </ul>
 * The cache has a fixed size (which can be configured with the {@link #setCacheLength(int)} method. The default
 * size is 250, which will have the last 5 seconds of motion at 20ms/commandCycle, and should be reasonable
 * for competition code.
 * <p>
 * The change in the position of the robot since the last target data is found
 * using {@link #getRobotRelativePositionSince(double)} where the {@code sinceTime} is the time of the last processed
 * target image, and this reports the estimated change in field position after {@code sinceTime}. Please note the
 * following assumptions about this relative position:
 * <ul>
 *     <li>The robot is locked into an expected heading when targeting starts, so any rotation speeds have been set
 *     by the PID loop trying to hold the heading constant, and are ignored.</li>
 *     <li>Since the expected heading is locked, the relative position is with respect to the the position at
 *     {@code sinceTime} assuming the robot was in the expected heading, i.e. The
 *     {@link RobotRelativePosition#forward}</li> is along the expected heading; the
 *     {@link RobotRelativePosition#strafe}</li> is 90&deg; to the expected heading; the
 *     {@link RobotRelativePosition#heading}</li> will be set to the expected heading; and,
 *     {@link RobotRelativePosition#cacheOverrun}</li> will normally be {@code false}, but, if it is {@code true}
 *     it means the {@code sinceTime} is before the oldest entry in the cache, and anything else in
 *     {@link RobotRelativePosition} is invalid.
 * </ul>
 *
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
        double timeStamp;
        AngleConstantD actualHeading;
        AngleConstantD expectedHeading;
        double forward;
        double strafe;
        double rotation;

        void set(double timeStamp, AngleConstantD actualHeading, AngleConstantD expectedHeading,
                 double forward, double strafe, double rotation) {
            this.timeStamp = timeStamp;
            this.actualHeading = actualHeading;
            this.expectedHeading = expectedHeading;
            this.forward = forward;
            this.strafe = strafe;
            this.rotation = rotation;
        }

        public double getTimeStamp() {
            return timeStamp;
        }

        public AngleConstantD getActualHeading() {
            return actualHeading;
        }
        public AngleConstantD getExpectedHeading() {
            return expectedHeading;
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

    // This is the actual cache of drive commands.
    ControlRequest[] controlRequests = null;
    int cacheLength;
    int mostRecentControlRequest;

    // This is the swerve drive subsystem the cache sits on top of, and the swerve geometry and performance
    // data the cache uses to predict current robot position from detected past positions.
    private DriveSubsystem driveSubsystem = null;
    private DriveMode driveMode = DriveMode.FIELD_RELATIVE;
    private double driveLength = 0.0;
    private double driveWidth = 0.0;
    private double maxMetersPerSec = 3.136;
    private double maxRadiansPerSec = 3.136;

    // logging the cache data for analysis, testing, and tuning
    public static final String SPEED_CACHE_LOG_NAME = "speedCache";
    boolean logSpeedCache = false;
    StringLogEntry speedCacheLog = null;

    // The factors for tuning the speed cache position projections
    public double scaleForward = 1.0;
    public double scaleStrafe = 1.0;
    public double phase = 0.0;
    /**
     *
     */
    private SpeedCachedSwerve() {
        // the constructor does nothing, except setup a default 5 second cache.
        setCacheLength(250);
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
            // So this is not quite as simple as it seems, primarily because of testing/tuning scenarios. Let's
            // elaborate on that:
            // * In competition the cache is used because we have sensed information from the past (april tag or other
            //   sensing that has latency) giving us a robot position at some time in the past. From that we want to
            //   project/guess the current position so we can use that to drive a responsive position PID. In that
            //   case, the mostRecentControlRequest is now, and we want the change between the sinceTime and now.
            // * In tuning/testing the cache is loaded with test-generated positions, so, the current time is generated
            //   relative to the data in the cache and is probably way before the last time in the cache.
            if (currentTime > controlRequests[backIndex].timeStamp) {
                // OK, this after the sinceTime, and before the currentTime, the current time represents either the
                // start of the interval we are interested in, or, the time of the last control request.
                ControlRequest controlRequest = controlRequests[backIndex];
                double deltaTime = currentTime - controlRequest.timeStamp;
                double deltaForward = deltaTime * controlRequest.forward * maxMetersPerSec;
                double deltaStrafe = deltaTime * controlRequest.strafe * maxMetersPerSec;;
                // correct the forward and strafe for the deviation from the expected heading
                AngleD headingDelta = new AngleD(controlRequest.actualHeading).subtract(controlRequest.expectedHeading);
                forward += (deltaForward * headingDelta.cos()) - (deltaStrafe * headingDelta.sin());
                strafe += (deltaForward * headingDelta.sin()) + (deltaStrafe * headingDelta.cos());
                // the expected heading is set at targeting so, the heading here is the expected heading
                headingRadians = controlRequest.expectedHeading.getRadians();
                currentTime = controlRequest.timeStamp;
            }
            if ((backIndex = nextBackIndex(backIndex)) == -1) {
                // There are not enough entries in the cache
                cacheOverrun = true;
                break;
            }
        }
        return new RobotRelativePosition(forward, strafe,
                new AngleD(AngleUnit.RADIANS, headingRadians), sinceTime, cacheOverrun);
    }

    @Nullable
    AngleD getExpectedHeadingDeltaAt(double time) {
        double nextTime;
        AngleD nextDelta = new AngleD();
        double lastTime = controlRequests[mostRecentControlRequest].timeStamp;
        if (time > lastTime) {
            // a strange situation where the requested time is after (more recent) than the most recently
            // recorded request (i.e. the time we are looking for is after our last recorded request). This
            // is a handling conundrum - if you want the current heading, talk to the NavX, you can't get it
            // here.
            throw new IllegalArgumentException("You are asking for newer heading information than what is in" +
                    " the speed cache. Please query the NavX for current heading information instead of the" +
                    " speed cache.");
        }
        AngleD lastDelta = new AngleD(controlRequests[mostRecentControlRequest].actualHeading).subtract(
                controlRequests[mostRecentControlRequest].expectedHeading);
        boolean cacheOverrun = false;
        int backIndex = nextBackIndex(mostRecentControlRequest);
        while (true) {
            nextTime = lastTime;
            nextDelta.setValue(lastDelta);
            lastTime = controlRequests[backIndex].timeStamp;
            lastDelta.setValue( AngleUnit.RADIANS,
                    (controlRequests[backIndex].actualHeading.getRadians() -
                            controlRequests[backIndex].expectedHeading.getRadians()));
            if (lastTime <= time) {
                // So we are now at the point where the lastTime is before the requested time, and the nextTime
                // is after the requested time. interpolate the heading delta between the
                return new AngleD(AngleUnit.RADIANS, lastDelta.getRadians() +
                        ((time - lastTime)/(nextTime-lastTime)) * (nextDelta.getRadians() - lastDelta.getRadians()));
            }

            if ((backIndex = nextBackIndex(backIndex)) == -1) {
                // There are not enough entries in the cache
                break;
            }
        }
        return null;
    }

    private int nextBackIndex(int currentBackIndex) {
        currentBackIndex--;
        if (currentBackIndex < 0) {
            currentBackIndex = cacheLength - 1;
        }
        // Check for overflow - i.e. we've wrapped around and are back to the last entry we put in the
        // cache, or, we've gone past the the first entry in the cache.
        return ((currentBackIndex == mostRecentControlRequest) ||
                (null == controlRequests[currentBackIndex].expectedHeading)) ? -1 : currentBackIndex;
    }

    /**
     * Adds a control request to the cache. This is a package function where the timestamp is specified, so the
     * cache can be loaded for test scenarios.
     *
     * @param timestamp       The <i>current time</i>, which is the FPGA timestamp (in seconds) during match play,
     *                        but will be assigned as required for testing.
     * @param actualHeading   The actual field heading of the robot when this method is called.
     * @param expectedHeading The expected field heading of the robot when this method is called.
     * @param forward         Forward speed (-1.0 to 1.0)
     * @param strafe          Strafe speed (-1.0 to 1.0)
     * @param rotation        Rotation speed (-1.0 to 1.0)
     */
    void addControlRequest(double timestamp, AngleConstantD actualHeading, AngleConstantD expectedHeading,
                           double forward, double strafe, double rotation) {
        // increment the index to the array of cached control requests
        mostRecentControlRequest++;
        if (mostRecentControlRequest >= cacheLength) {
            mostRecentControlRequest = 0;
        }
        // save this request with a timestamp
        controlRequests[mostRecentControlRequest].set(timestamp, actualHeading, expectedHeading,
                forward, strafe, rotation );
        if (logSpeedCache) {
            speedCacheLog.append(String.format("%.4f,%.5f,%.5f,%.5f,%.5f,%.5f",
                    timestamp, actualHeading.getRadians(), expectedHeading.getRadians(),
                    forward, strafe, rotation));
        }
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

    public void setLogging(boolean on) {
        logSpeedCache = on;
        if (logSpeedCache) {
            if (null == speedCacheLog) {
                speedCacheLog = new StringLogEntry(DataLogManager.getLog(), SPEED_CACHE_LOG_NAME);
            }
        }
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
        NavX.HeadingInfo headingInfo = NavX.getInstance().getHeadingInfo();
        addControlRequest(Timer.getFPGATimestamp(), headingInfo.heading, headingInfo.expectedHeading,
                forward, strafe, rotation
        );
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

