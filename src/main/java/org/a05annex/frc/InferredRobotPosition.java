package org.a05annex.frc;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.a05annex.frc.subsystems.SpeedCachedSwerve;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * This is similar to the {@link RobotPosition} class, however it implements the {@link SpeedCachedSwerve} to improve accuracy
 * of the calculated position when the robot is in motion.
 */
public class InferredRobotPosition extends RobotPosition {
    /**
     * Stores the last valid InferredRobotPosition
     */
    private static InferredRobotPosition lastValidIRP = new InferredRobotPosition();

    private static double lastValidIRPTime = 0.0;

    private static boolean isCachingPaused = false;

    /**
     * The timestamp of the {@link RobotPosition#pipelineResult} or 0.0 if targeting is not possible.
     */
    public final double timestamp;
    /**
     * An {@link InferredRobotPosition} instance that is invalid and cannot be used for targeting. This is a separate,
     * public variable to optimize memory and so that all classes can compare a received {@link InferredRobotPosition}
     * to this instance to determine if it is invalid.
     */
    public static final InferredRobotPosition INVALID_IRP = new InferredRobotPosition();

    /**
     * Default constructor which sets to default values. Used when isValid is false and targeting is impossible.
     */
    private InferredRobotPosition() {
        super();
        timestamp = 0.0;
    }

    /**
     * Paramaterized constructor to create a new {@link InferredRobotPosition}.
     *
     * @param isValid        Indicates if the position data is valid.
     * @param isNew          Indicates if the data is from the latest observation.
     * @param x              The x-coordinate of the position.
     * @param y              The y-coordinate of the position.
     * @param pipelineResult The latest camera pipeline result.
     * @param tagSet         The set of AprilTags used.
     * @param timestamp      the timestamp of {@link #pipelineResult}. This is extracted to its own variable since it is used
     *                       frequently enough when interacting with the cache.
     */
    private InferredRobotPosition(boolean isValid, boolean isNew, double x, double y, PhotonPipelineResult pipelineResult, A05Constants.AprilTagSet tagSet, double timestamp) {
        super(isValid, isNew, x, y, pipelineResult, tagSet);
        this.timestamp = timestamp;
    }

    /**
     * Creates a {@link InferredRobotPosition} which defines the robot's position relative to a target AprilTagSet while
     * also inferring the robots change in position since the {@link #pipelineResult} was taken using the {@link SpeedCachedSwerve}.
     *
     * @param tagSetKey The key String identifying the {@link org.a05annex.frc.A05Constants.AprilTagSet}.
     * @return A {@link InferredRobotPosition} representing the robot's current position relative to the target
     * {@link org.a05annex.frc.A05Constants.AprilTagSet} with the cache's predicted extra movement since the
     * {@link #pipelineResult} was captured.
     */
    @SuppressWarnings("unused")
    public static InferredRobotPosition getRobotPosition(String tagSetKey) {
        return getRobotPosition(A05Constants.aprilTagSetDictionary.get(tagSetKey));
    }

    /**
     * Creates a {@link InferredRobotPosition} which defines the robot's position relative to a target AprilTagSet while
     * also inferring the robots change in position since the {@link #pipelineResult} was taken using the {@link SpeedCachedSwerve}.
     *
     * @param tagSet The {@link org.a05annex.frc.A05Constants.AprilTagSet} to use for positioning.
     * @return A {@link InferredRobotPosition} representing the robot's current position relative to the target
     * {@link org.a05annex.frc.A05Constants.AprilTagSet} with the cache's predicted extra movement since the
     * {@link #pipelineResult} was captured.
     */
    public static InferredRobotPosition getRobotPosition(A05Constants.AprilTagSet tagSet) {
        RobotPosition robotPosition = RobotPosition.getRobotPosition(tagSet);

        // Verify that isValid is true and the SpeedCachedSwerve is still caching to avoid throwing
        // a NullPointerException or IllegalArgumentException
        if(!robotPosition.isValid) {
            return tagSet == lastValidIRP.tagSet ? cacheIncrementLastIRP() : INVALID_IRP;
        }
        if(isCachingPaused) {
            return lastValidIRP;
        }

        double timestamp = robotPosition.pipelineResult.getTimestampSeconds();

        try {
            SpeedCachedSwerve.RobotRelativePosition scsRobotRelativePosition = SpeedCachedSwerve.getInstance().getRobotRelativePositionSince(timestamp);
            if(scsRobotRelativePosition.cacheOverrun) {
                return INVALID_IRP;
            }

            TruePosition inferredPos = new TruePosition(robotPosition.x - scsRobotRelativePosition.forward, robotPosition.y - scsRobotRelativePosition.strafe);

            lastValidIRP = new InferredRobotPosition(true, robotPosition.isNew, inferredPos.x(), inferredPos.y(), robotPosition.pipelineResult, robotPosition.tagSet, timestamp);
            lastValidIRPTime = Timer.getFPGATimestamp();
            return lastValidIRP;
        } catch (Exception e) {
            DriverStation.reportWarning("Getting a new InferredRobotPosition produced an error. The last valid " +
                    "IRP was sent instead. InferredRobotPosition encountered this error: " + e.getMessage(), true);
            return lastValidIRP;
        }
    }

    /**
     * Indicate pausing of the caching of the {@link SpeedCachedSwerve} to avoid errors from disabled periodic telemetry.
     */
    public static void pauseCaching() {
        isCachingPaused = true;
    }
    /**
     * Indicate that the {@link SpeedCachedSwerve} has resumed caching after it has been paused.
     */
    public static void resumeCaching() {
        isCachingPaused = false;
    }
    /**
     * Returns whether the {@link SpeedCachedSwerve} is receiving new data or if caching is paused.
     *
     * @return true if the caching is paused, false otherwise.
     */
    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public static boolean isCachingPaused() {
        return isCachingPaused;
    }

    /**
     * Increments the last valid {@link InferredRobotPosition} with the cache's predicted extra movement since the
     * {@link #lastValidIRP} was calculated.
     *
     * @return The last valid {@link InferredRobotPosition} with the cache's predicted extra movement since the
     * {@link #lastValidIRP} was calculated, or {@link #INVALID_IRP} if the last valid {@link InferredRobotPosition}
     * is invalid or the cache has overrun.
     */
    public static InferredRobotPosition cacheIncrementLastIRP() {
        if(!lastValidIRP.isValid) {
            lastValidIRP = INVALID_IRP;
            return lastValidIRP;
        }

        try {
            SpeedCachedSwerve.RobotRelativePosition scsRobotRelativePosition = SpeedCachedSwerve.getInstance().getRobotRelativePositionSince(lastValidIRPTime);

            if(scsRobotRelativePosition.cacheOverrun) {
                lastValidIRP = INVALID_IRP;
                return lastValidIRP;
            }

            TruePosition inferredPos = new TruePosition(lastValidIRP.x - scsRobotRelativePosition.forward, lastValidIRP.y - scsRobotRelativePosition.strafe);

            lastValidIRP = new InferredRobotPosition(lastValidIRP.isValid, lastValidIRP.isNew, inferredPos.x(), inferredPos.y(), lastValidIRP.pipelineResult, lastValidIRP.tagSet, lastValidIRP.timestamp);
            lastValidIRPTime = Timer.getFPGATimestamp();
            return lastValidIRP;
        } catch (Exception e) {
            DriverStation.reportWarning("Getting a new InferredRobotPosition produced an error. The last valid " +
                    "IRP was sent instead. InferredRobotPosition encountered this error: " + e.getMessage(), true);
            lastValidIRP = INVALID_IRP;
            return lastValidIRP;
        }
    }
}

