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
 * to a field position at the current time. The use case is with vision systems that take some time (typically
 * 20 to 120ms) to analyze the field targets and report their position relative to the target, and it follows
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
 *     {@link RobotRelativePosition#forward} is along the expected heading; the
 *     {@link RobotRelativePosition#strafe} is 90&deg; to the expected heading; the
 *     {@link RobotRelativePosition#heading} will be set to the expected heading; and,
 *     {@link RobotRelativePosition#cacheOverrun} will normally be {@code false}, but, if it is {@code true}
 *     it means the {@code sinceTime} is before the oldest entry in the cache, and anything else in
 *     {@link RobotRelativePosition} is invalid.</li>
 * </ul>
 * <b>The Forward Projection Algorithm</b>
 * So the question is, now that we have cached all this data about robot heading and what we have asked to robot
 * to do at every command, how do we project the relative movement from a known field position in the past;
 * specifically, we have a past position relative to an april tag, and we want to know the current position
 * relative to that tag.
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
     * The record of a {@link ISwerveDrive#swerveDriveComponents(double, double, double)} function call annotated with
     * the match timestamp, expected robot heading, and actual robot heading at the time the function was called.
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

        /**
         * Get the timestamp at the time the {@link ISwerveDrive#swerveDriveComponents(double, double, double)}
         * function was called.
         *
         * @return The timestamp.
         */
        public double getTimeStamp() {
            return timeStamp;
        }

        /**
         * Get the actual heading at the time the {@link ISwerveDrive#swerveDriveComponents(double, double, double)}
         * function was called.
         *
         * @return The actual heading.
         */

        public AngleConstantD getActualHeading() {
            return actualHeading;
        }

        /**
         * Get the expected heading at the time the {@link ISwerveDrive#swerveDriveComponents(double, double, double)}
         * function was called.
         *
         * @return The expected heading.
         */
        public AngleConstantD getExpectedHeading() {
            return expectedHeading;
        }

        /**
         * Get the forward argument to the {@link ISwerveDrive#swerveDriveComponents(double, double, double)}
         * function call.
         *
         * @return The forward argument.
         */
        public double getForward() {
            return forward;
        }

        /**
         * Get the strafe argument to the {@link ISwerveDrive#swerveDriveComponents(double, double, double)}
         * function call.
         *
         * @return The strafe argument.
         */
        public double getStrafe() {
            return forward;
        }

        /**
         * Get the rotation argument to the {@link ISwerveDrive#swerveDriveComponents(double, double, double)}
         * function call.
         *
         * @return The rotation argument.
         */
        public double getRotation() {
            return forward;
        }

    }

    /**
     * This class specifies a robot position relative to a starting robot position defines the reference
     * coordinate axis system origin and orientation.
     */
    public static class RobotRelativePosition {
        /**
         * The distance forward in the reference axis system.
         */
        public double forward;
        /**
         * The distance to the right in the reference axis system.
         */
        public double strafe;
        /**
         * the heading relative to the reference axis system.
         */
        public AngleD heading;
        /**
         * The match (field) timestamp for this position.
         */
        public double timeStamp;
        /**
         * {@code true} if the requested position time is far enough into the future that there were not enough
         * entries in the cache to predict the position, i.e. this position is invalid. {@code false} if this is
         * a valid position prediction.
         */
        public boolean cacheOverrun;

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
    /**
     * The name of log entries from this component.
     */
    public static final String SPEED_CACHE_LOG_NAME = "speedCache";
    boolean logSpeedCache = false;
    StringLogEntry speedCacheLog = null;

    // The factors for tuning the speed cache position projections
    /**
     * As we were investigating sources of error in the {@link SpeedCachedSwerve} predictions, we identified scaling
     * as a possible source of error. This scaling factor was introduced so that we could explore whether scaling
     * could be used to reduce errors.
     */
    private double scaleForward = 1.0;
    /**
     * As we were investigating sources of error in the {@link SpeedCachedSwerve} predictions, we identified scaling
     * as a possible source of error. This scaling factor was introduced so that we could explore whether scaling
     * could be used to reduce errors.
     */
    private double scaleStrafe = 1.0;
    /**
     * As we were investigating sources of error in the {@link SpeedCachedSwerve} predictions, we <i>phase</i> as
     * a possible source of error. In this context, <i>phase</i> is where in a command cycle the drive gets to the
     * newly assigned speeds. A phase of 0.0 is that the drive instantly sets to the newly assigned speeds as they
     * are assigned. A phase of 1.0 is that they do not get to the newly assigned speeds until the next command cycle
     * immediately before the new speeds are assigned.
     */
    private double phase = 0.0;

    /**
     * We have observed that the timestamp from the PhotonVisionFrames is not the real time that the photo was taken, so
     * this offset is intended to correct for that allowing for a more accurate cache prediction.
     */
    private double latencyOffset = 0.0;


    private SpeedCachedSwerve() {
        // the constructor does nothing, except set up a default 5 second cache.
        setCacheLength(250);
    }

    /**
     * Get the most recent control request.
     * @return (readonly) the most recent control request.
     */
    ControlRequest getMostRecentControlRequest() {
        return controlRequests[mostRecentControlRequest];
    }

    /**
     * <b>DO NOT CALL THIS METHOD UNLESS YOU ARE EXPLORING FUTURE POSITION PREDICTION ERRORS.</b> As we were
     * investigating sources of error in the {@link SpeedCachedSwerve} predictions, we identified scaling
     * as a possible source of error. This method resets both the forward and strafe scale.
     *
     * @param scale The scale by default is 1.0, This lets you specify an alternate forward and strafe scaling.
     */
    public void setMaxVelocityScale(double scale) {
        this.scaleForward = scale;
        this.scaleStrafe = scale;
    }

    /**
     * <b>DO NOT CALL THIS METHOD UNLESS YOU ARE EXPLORING FUTURE POSITION PREDICTION ERRORS.</b> As we were
     * investigating sources of error in the {@link SpeedCachedSwerve} predictions, we identified scaling
     * as a possible source of error. This method resets both the forward and strafe scale.
     *
     * @param forwardScale The scale by default is 1.0, This lets you specify an alternate forward scaling.
     */
    public void setMaxForwardScale(double forwardScale) {
        this.scaleForward = forwardScale;
    }

    /**
     * Get the current forward scale factor.
     *
     * @return The forward scale factor.
     */
    public double getMaxForwardScale() {
        return scaleForward;
    }

    /**
     * <b>DO NOT CALL THIS METHOD UNLESS YOU ARE EXPLORING FUTURE POSITION PREDICTION ERRORS.</b> As we were
     * investigating sources of error in the {@link SpeedCachedSwerve} predictions, we identified scaling
     * as a possible source of error. This method resets both the forward and strafe scale.
     *
     * @param scaleStrafe The scale by default is 1.0, this lets you specify an alternate strafe scaling.
     */
    public void setMaxStrafeScale(double scaleStrafe) {
        this.scaleStrafe = scaleStrafe;
    }

    /**
     * Get the current strafe scale factor.
     *
     * @return The strafe scale factor.
     */
    public double getMaxStrafeScale() {
        return scaleStrafe;
    }

    /**
     * Set the phase for future position prediction.
     *
     * @param phase The phase, from 0.0 to 1.0
     */
    public void setPhase(double phase) {
        this.phase = Utl.clip(phase,0.0,1.0);
    }

    /**
     * Gets the phase.
     *
     * @return The phase.
     */
    public double getPhase() {
        return phase;
    }

    public void setLatencyOffset(double latencyOffset) {
        this.latencyOffset = latencyOffset;
    }

    public double getLatencyOffset() {
        return latencyOffset;
    }

    /**
     * Get the robot position now, relative to where the robot was at the specified time. For example suppose the robot
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
     * @param targetTime The <i>current time</i>, which is the FPGA timestamp (in seconds) during match play,
     *                   but will be assigned as required for testing.
     * @param sinceTime  The time (FPGA timestamp in seconds) from which you want to know the robot's new
     *                   position. This is typically the time of the last known position through an april tag
     *                   or similar position sensor.
     * @return The estimated robot position at {@code sinceTime}
     */
    @NotNull
    RobotRelativePosition getRobotRelativePositionSince(double targetTime, double sinceTime) {
        sinceTime -= latencyOffset;
        int backIndex = mostRecentControlRequest;
        ControlRequest lastControlRequest = controlRequests[backIndex];
        double lastTime = lastControlRequest.timeStamp;
        if (targetTime > (lastTime + 0.045)) {
            // a strange situation where the requested time is far after (more than 40ms or 2 control cycles
            // more recent) than the most recently
            // recorded request (i.e. the time we are looking for is after our last recorded request). This
            // is a handling conundrum - if you want the current heading, talk to the NavX, you can't get it
            // here.
            throw new IllegalArgumentException("You are asking for a position projected more than 2 control" +
                    " cycles (40ms) into the future, which this cache cannot do. If you want to know where the" +
                    " robot is now, you need a different strategy.");
        }
        double nextTime;
        ControlRequest nextControlRequest;
        int forwardIndex;
        ControlRequest forwardControlRequest;
        double forwardTime;
        RobotRelativePosition position = new RobotRelativePosition(0.0, 0.0,
                new AngleD(AngleUnit.RADIANS, 0.0), sinceTime, false);
        // we need a bit of history before the sinceTime, so we know what the robot is doing at sinceTime. ideally
        // we need the speed info from two steps before - because we expect the robot to have achieved that speed by
        // the step before - and then the fun begins.
        double sinceTimePositionInStep;
        while (true) {
            nextTime = lastTime;
            nextControlRequest = lastControlRequest;
            forwardIndex = backIndex;
            if ((backIndex = nextBackIndex(backIndex)) == -1) {
                // There are not enough entries in the cache
                position.cacheOverrun = true;
                return position;
            }
            lastControlRequest = controlRequests[backIndex];
            lastTime = lastControlRequest.timeStamp;
            if (lastTime <= sinceTime) {
                // So we are now at the point where the lastTime is before the requested time. Depending on phase,
                // we may need info from the next point back.
                sinceTimePositionInStep = ((sinceTime - lastTime) / (nextTime - lastTime));
                if (sinceTimePositionInStep >= phase) {
                    // since position in the interval is after the phase which shifts the velocities of the swerve to
                    // the newly set velocities.
                    break;
                }
            }
        }
        // OK, we have the start index and we will start accumulating from there - make sure there is a next forward
        // index. It not, the camera latency is really low, and the 'sinceTime' is after the last recorded
        // swerve command. Specifically, a command was recorded, and a new April Tag location comes in after that
        // command and before the next
        AngleD headingDelta;
        forwardIndex = nextForwardIndex(forwardIndex);
        if (forwardIndex == -1) {
            forwardControlRequest = null;
            forwardTime = targetTime;
        } else {
            forwardControlRequest = controlRequests[forwardIndex];
            forwardTime = forwardControlRequest.timeStamp;
        }
        // the first step is the special case where the sinceTime happens sometime within an interval between
        // two recorded commands and we want to accumulate just the part after the since time.
        // so we need to determine the speed profile (based on phase) for the fist step and get things started.
        if (sinceTimePositionInStep < 1.0) {
            // sinceTimePositionInStep is greater than phase, so the robot is traveling from backIndex to the
            // next forward index, and it has reached the backIndex speeds
            double timeAtSpeed = (1.0 - sinceTimePositionInStep) * (nextTime - lastTime);
            double deltaForward = timeAtSpeed * lastControlRequest.forward * maxMetersPerSec;
            double deltaStrafe = timeAtSpeed * lastControlRequest.strafe * maxMetersPerSec;
            headingDelta = new AngleD(nextControlRequest.actualHeading).subtract(nextControlRequest.expectedHeading);
            position.forward += ((deltaForward * headingDelta.cos()) - (deltaStrafe * headingDelta.sin())) * scaleForward;
            position.strafe += ((deltaForward * headingDelta.sin()) + (deltaStrafe * headingDelta.cos())) * scaleStrafe;
        } else {
            // sinceTimePositionInStep > 1.0 which means we are an interval earlier than the interval that includes
            // the sinceTime, and that sinceTime is before the phase in the next step, so the robot is going at the
            // lastControlRequest velocities between sinceTime and the phase, then we step forward for the right
            // control request for post-phase movement.
            //
            // this is the part before the phase, where the velocity targets were set at the beginning of the
            // previous interval.
            double timeAtSpeed = (phase - (sinceTimePositionInStep - 1.0)) * (forwardTime - nextTime);
            double deltaForward, deltaStrafe;
            if (timeAtSpeed > 0.0) {
                deltaForward = timeAtSpeed * lastControlRequest.forward * maxMetersPerSec;
                deltaStrafe = timeAtSpeed * lastControlRequest.strafe * maxMetersPerSec;
                headingDelta = new AngleD(nextControlRequest.actualHeading).subtract(nextControlRequest.expectedHeading);
                position.forward += ((deltaForward * headingDelta.cos()) - (deltaStrafe * headingDelta.sin())) * scaleForward;
                position.strafe += ((deltaForward * headingDelta.sin()) + (deltaStrafe * headingDelta.cos())) * scaleStrafe;
            }
            // this is the part after the phase where the velocities have reached those set at the start of the
            // interval and the heading delta from the forward
            timeAtSpeed = (1.0 - (((sinceTimePositionInStep - 1.0) > phase) ? (sinceTimePositionInStep - 1.0) : phase)) * (forwardTime - nextTime);
            deltaForward = timeAtSpeed * nextControlRequest.forward * maxMetersPerSec;
            deltaStrafe = timeAtSpeed * nextControlRequest.strafe * maxMetersPerSec;
            headingDelta = (null == forwardControlRequest) ? new AngleD(AngleUnit.RADIANS, 0.0) :
                    new AngleD(forwardControlRequest.actualHeading).subtract(forwardControlRequest.expectedHeading);
            position.forward += ((deltaForward * headingDelta.cos()) - (deltaStrafe * headingDelta.sin())) * scaleForward;
            position.strafe += ((deltaForward * headingDelta.sin()) + (deltaStrafe * headingDelta.cos())) * scaleStrafe;
            // now move an interval forward
            lastControlRequest = nextControlRequest;
            lastTime = nextTime;
            nextControlRequest = forwardControlRequest;
            nextTime = forwardTime;
            if (forwardIndex != -1) {
                forwardIndex = nextForwardIndex(forwardIndex);
                forwardControlRequest = controlRequests[forwardIndex];
                forwardTime = forwardControlRequest.timeStamp;
            }
        }

        if (forwardIndex != -1) {
            // now step through until the end of the next interval is
            while (forwardTime < targetTime) {
                double timeAtSpeed = phase * (forwardTime - nextTime);
                double deltaForward = timeAtSpeed * lastControlRequest.forward * maxMetersPerSec;
                double deltaStrafe = timeAtSpeed * lastControlRequest.strafe * maxMetersPerSec;
                headingDelta = new AngleD(nextControlRequest.actualHeading).subtract(nextControlRequest.expectedHeading);
                position.forward += ((deltaForward * headingDelta.cos()) - (deltaStrafe * headingDelta.sin())) * scaleForward;
                position.strafe += ((deltaForward * headingDelta.sin()) + (deltaStrafe * headingDelta.cos())) * scaleStrafe;
                // this is the part after the phase where the velocities have reached those set at the start of the
                // interval and the heading delta from the forward
                timeAtSpeed = (1.0 - phase) * (forwardTime - nextTime);
                deltaForward = timeAtSpeed * nextControlRequest.forward * maxMetersPerSec;
                deltaStrafe = timeAtSpeed * nextControlRequest.strafe * maxMetersPerSec;
                headingDelta = new AngleD(forwardControlRequest.actualHeading).subtract(forwardControlRequest.expectedHeading);
                position.forward += ((deltaForward * headingDelta.cos()) - (deltaStrafe * headingDelta.sin())) * scaleForward;
                position.strafe += ((deltaForward * headingDelta.sin()) + (deltaStrafe * headingDelta.cos())) * scaleStrafe;
                // now move an interval forward
                forwardIndex = nextForwardIndex(forwardIndex);
                lastControlRequest = nextControlRequest;
                lastTime = nextTime;
                nextControlRequest = forwardControlRequest;
                nextTime = forwardTime;
                if (forwardIndex == -1) {
                    forwardControlRequest = null;
                    forwardTime = (targetTime > (nextTime + 0.02)) ? (nextTime + 0.02) : targetTime;
                    break;
                }
                forwardControlRequest = controlRequests[forwardIndex];
                forwardTime = forwardControlRequest.timeStamp;
            }
        }

        // OK, the forward point is ahead of (at a newer time than) the target time
        if (nextTime < targetTime) {
            double targetTimePositionInStep = (targetTime - nextTime) / (forwardTime - nextTime);
            double timeAtSpeed = ((phase < targetTimePositionInStep) ? phase : targetTimePositionInStep) *
                    (forwardTime - nextTime);
            double deltaForward = timeAtSpeed * lastControlRequest.forward * maxMetersPerSec;
            double deltaStrafe = timeAtSpeed * lastControlRequest.strafe * maxMetersPerSec;
            headingDelta = new AngleD(nextControlRequest.actualHeading).subtract(nextControlRequest.expectedHeading);
            position.forward += ((deltaForward * headingDelta.cos()) - (deltaStrafe * headingDelta.sin())) * scaleForward;
            position.strafe += ((deltaForward * headingDelta.sin()) + (deltaStrafe * headingDelta.cos())) * scaleStrafe;
            if (targetTimePositionInStep > phase) {
                // this is the part after the phase where the velocities have reached those set at the start of the
                // interval and the heading delta from the forward
                timeAtSpeed = (targetTimePositionInStep - phase) * (forwardTime - nextTime);
                deltaForward = timeAtSpeed * nextControlRequest.forward * maxMetersPerSec;
                deltaStrafe = timeAtSpeed * nextControlRequest.strafe * maxMetersPerSec;
                if (null != forwardControlRequest) {
                    headingDelta = new AngleD(forwardControlRequest.actualHeading).
                            subtract(forwardControlRequest.expectedHeading);
                }
                position.forward += ((deltaForward * headingDelta.cos()) - (deltaStrafe * headingDelta.sin())) * scaleForward;
                position.strafe += ((deltaForward * headingDelta.sin()) + (deltaStrafe * headingDelta.cos())) * scaleStrafe;
            }
        }

        return position;
    }

    /**
     * Returns the delta between the expected robot heading and the actual robot heading at the requested time. The
     * use case is when the heading for some sensor (like vision) has a latency and requires information about the
     * robot heading sometime in the past. This returned delta is a linear interpolation approximation between the
     * cached heading in the command cycles before and after the requested time. <b>NOTE:</b> there are 2 conditions
     * where this method fails.
     * <ul>
     *     <li><b>cache Overflow</b> - There is not enough history in the cache to satisfy this request, in which
     *     case {@code null} is returned.</li>
     *     <li>{@link IllegalArgumentException} - the requested {@code time} is more recent than the last cache
     *     entry, please used {@link NavX#getHeadingInfo()} to get the most recent (current) heading.</li>
     * </ul>
     *
     * @param time The time (FPGA timestamp in seconds) at which you want to know the heading delta.
     * @return Returns the heading delta, of {@code null} if the data in the cache does not extend back to the
     * specified time.
     * @throws IllegalArgumentException Thrown if the requested {@code time} is more recent than the last cache
     *                                  entry, please used {@link NavX#getHeadingInfo()} to get the most recent (current) heading.
     */
    @Nullable
    AngleD getExpectedHeadingDeltaAt(double time) {
        time -= latencyOffset;
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
        int backIndex = nextBackIndex(mostRecentControlRequest);
        while (true) {
            nextTime = lastTime;
            nextDelta.setValue(lastDelta);
            lastTime = controlRequests[backIndex].timeStamp;
            lastDelta.setValue(controlRequests[backIndex].actualHeading).subtract(
                    controlRequests[backIndex].expectedHeading);
            if (lastTime <= time) {
                // So we are now at the point where the lastTime is before the requested time, and the nextTime
                // is after the requested time. interpolate the heading delta between the
                return new AngleD(AngleUnit.RADIANS, lastDelta.getRadians() +
                        ((time - lastTime) / (nextTime - lastTime)) * (nextDelta.getRadians() - lastDelta.getRadians()));
            }

            if ((backIndex = nextBackIndex(backIndex)) == -1) {
                // There are not enough entries in the cache
                break;
            }
        }
        return null;
    }

    /**
     * For the current index, get the index of the entry before the current one.
     *
     * @param currentIndex The current index
     * @return returns the index of the entry before this one, or -1 if there is a cache overflow.
     */
    private int nextBackIndex(int currentIndex) {
        currentIndex--;
        if (currentIndex < 0) {
            currentIndex = cacheLength - 1;
        }
        // Check for overflow - i.e. we've wrapped around and are back to the last entry we put in the
        // cache, or, we've gone past the the first entry in the cache.
        return ((currentIndex == mostRecentControlRequest) ||
                (null == controlRequests[currentIndex].expectedHeading)) ? -1 : currentIndex;
    }

    /**
     * For the current index, get the index of the entry after the current one.
     *
     * @param currentIndex The current index
     * @return returns the index of the entry after this one, or -1 if there is a cache overflow.
     */
    private int nextForwardIndex(int currentIndex) {
        if (currentIndex == mostRecentControlRequest) {
            return -1;
        }
        currentIndex++;
        if (currentIndex > (cacheLength - 1)) {
            currentIndex = 0;
        }
        return currentIndex;
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
                forward, strafe, rotation);
        if (logSpeedCache) {
            speedCacheLog.append(String.format("%.4f,%.5f,%.5f,%.5f,%.5f,%.5f",
                    timestamp, actualHeading.getRadians(), expectedHeading.getRadians(),
                    forward, strafe, rotation));
        }
    }

    /**
     * Set the cache length, which sets the maximum time interval for the cache to predict future positions. The
     * command cycle is 20ms, or 50 command cycles per second. The default initialization is a cache of length 250,
     * or about 5 seconds. When a new length is set all past cache contents are lost.
     *
     * @param cacheLength Set a new length for the cache.
     */
    public void setCacheLength(int cacheLength) {
        this.cacheLength = cacheLength;
        mostRecentControlRequest = -1;
        controlRequests = new ControlRequest[cacheLength];

        for (int i = 0; i < cacheLength; i++) {
            controlRequests[i] = new ControlRequest();
        }
    }

    /**
     * Report the length of the speed cache.
     *
     * @return The number of entries in the speed cache.
     */
    public int getCacheLength() {
        return cacheLength;
    }

    /**
     * Set the logging state for the speed cache. By default, logging is off.
     *
     * @param on {@code true} if the speed cache should be logging, {@code false} if it should not log.
     */
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
    /**
     * Ste the swerve drive subsystem that is supporting the speed cache. All {@link ISwerveDrive} commands to the
     * speed cache will be passed on to the {@code driveSubsystem}.
     *
     * @param driveSubsystem The underlying {@link DriveSubsystem}.
     */
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

