package org.a05annex.frc;

import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.jetbrains.annotations.NotNull;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Represents the robot's position calculated from the data provided by the camera and navigation systems.
 * Utilizes AprilTags to ascertain the position in the field relative to a specific tag.
 */
public class RobotPosition {

    /**
     * The {@link PhotonCameraWrapper} camera to use for targeting.
     */
    protected static PhotonCameraWrapper camera = null;

    /**
     * Pointer to the {@link NavX} instance.
     * This is used for acquiring the robot's current heading and other navigational data.
     */
    protected final static NavX navX = NavX.getInstance();

    /**
     * Flag to tell if the data contained in the {@link RobotPosition} is valid and real.
     * If false, the position data should be considered inaccurate.
     */
    public final boolean isValid;

    /**
     * Flag to tell if the data contained in the {@link RobotPosition} came from the latest {@link PhotonPipelineResult}.
     * This indicates whether the data is from a new set of observations.
     */
    public final boolean isNew;

    /**
     * The x-coordinate of the robot's position.
     */
    public final double x;

    /**
     * The y-coordinate of the robot's position.
     */
    public final double y;

    /**
     * The set of AprilTags used for determining the robot's position.
     */
    public final A05Constants.AprilTagSet tagSet;

    /**
     * The latest pipeline result from the camera, which is used for targeting and positioning.
     */
    public PhotonPipelineResult pipelineResult;

    /**
     * Constructor to create a RobotPosition with specific parameters.
     *
     * @param isValid Indicates if the position data is valid.
     * @param isNew Indicates if the data is from the latest observation.
     * @param x The x-coordinate of the position.
     * @param y The y-coordinate of the position.
     * @param pipelineResult The latest camera pipeline result.
     * @param tagSet The set of AprilTags used.
     */
    private RobotPosition(boolean isValid, boolean isNew, double x, double y, PhotonPipelineResult pipelineResult, A05Constants.AprilTagSet tagSet) {
        this.isValid = isValid;
        this.isNew = isNew;
        this.x = x;
        this.y = y;
        this.pipelineResult = pipelineResult;
        this.tagSet = tagSet;
    }

    /**
     * Default constructor creating an invalid RobotPosition instance.
     * This is used when targeting is not possible or no valid position can be calculated.
     */
    private RobotPosition() {
        this.isValid = false;
        this.isNew = false;
        this.x = 0.0;
        this.y = 0.0;
        this.pipelineResult = null;
        this.tagSet = null;
    }

    /**
     * Sets the camera to be used for robot positioning.
     * This method ensures that the camera is set only once to avoid reinitialization errors.
     *
     * @param camera The {@link PhotonCameraWrapper} to be used.
     * @throws IllegalStateException if the camera is set more than once.
     */
    public static void setCamera(@NotNull PhotonCameraWrapper camera) {
        if (RobotPosition.camera == null) {
            RobotPosition.camera = camera;
            return;
        }
        throw new IllegalStateException("Attempted to initialize Robot Position camera more than once");
    }

    /**
     * Creates a RobotPosition which defines the robot's position relative to a target AprilTagSet.
     *
     * @param tagSetKey The key String identifying the {@link org.a05annex.frc.A05Constants.AprilTagSet}.
     * @return A {@link RobotPosition} representing the robot's current position relative to the target {@link org.a05annex.frc.A05Constants.AprilTagSet}.
     */
    public static RobotPosition getRobotPosition(String tagSetKey) {
        A05Constants.AprilTagSet tagSet = A05Constants.aprilTagSetDictionary.get(tagSetKey);
        return getRobotPosition(tagSet);
    }

    /**
     * Creates a RobotPosition which defines the robot's position relative to a target AprilTagSet.
     *
     * @param tagSet The {@link org.a05annex.frc.A05Constants.AprilTagSet} to use for positioning.
     * @return A {@link RobotPosition} representing the robot's current position relative to the target {@link org.a05annex.frc.A05Constants.AprilTagSet}.
     */
    public static RobotPosition getRobotPosition(A05Constants.AprilTagSet tagSet) {
        if (!canTarget(tagSet)) {
            return new RobotPosition(); // Returns blank RobotPosition with isValid flag set to false
        }

        TruePosition pos = solveForTruePosition(tagSet);
        return new RobotPosition(true, camera.isTargetDataNew(tagSet), pos.x, pos.y, camera.getNewestFrameWithTarget(), tagSet);
    }

    /**
     * PhotonVision produces vectors relative to the robot heading, meaning that a robot facing a target and on an arc
     * around that target would have the same position data regardless of its location on the arc. <p>
     * This method solves for a tag relative position that is unaffected by the robot heading. This <em>True Position</em> of the
     * robot is found using the camera's data and the current heading.<p>
     * The actual calculations are performed by {@link #solveForTruePositionTestMethod(double, double, AngleD)}
     *
     * @param tagSet The set of AprilTags to use.
     * @return The calculated true position of the robot.
     */
    protected static @NotNull TruePosition solveForTruePosition(A05Constants.AprilTagSet tagSet) {
        double camX = camera.getXFromLastTarget(tagSet); // camera X is distance from target
        double camY = camera.getYFromLastTarget(tagSet); // camera Y is horizontal offset X

        AngleD headingDelta = navX.getHeadingInfo().getClosestHeading(tagSet.heading()).subtract(navX.getHeading()).cloneAngleD(); // Tag heading - current heading

        double[] output = solveForTruePositionTestMethod(camX, camY, headingDelta);
        return new TruePosition(output[0], output[1]);
    }

    /**
     * This method performs the calculations described in {@link #solveForTruePosition(A05Constants.AprilTagSet)}.<p>
     * This method extracts the calculations to allow for easier testing of the math.
     *
     * @param camX The distance from the target along the camera's x-axis.
     * @param camY The horizontal offset along the camera's y-axis.
     * @param headingDelta The difference in angle between the tag's heading and the current heading.
     * @return An array containing the true x and y coordinates.
     */
    static double[] solveForTruePositionTestMethod(double camX, double camY, AngleD headingDelta) {
        headingDelta.mult(-1.0);
        AngleD hypotenuseAngle = headingDelta.add(new AngleD().atan(camX / camY)).cloneAngleD();

        if (camY < 0.0) {
            hypotenuseAngle.add(AngleConstantD.DEG_180);
        }

        double hypotenuse = Math.sqrt(Math.pow(camX, 2) + Math.pow(camY, 2));
        double x = hypotenuseAngle.sin() * hypotenuse; // true X is distance from target
        double y = hypotenuseAngle.cos() * hypotenuse;

        return new double[]{x, y};
    }

    /**
     * Record class to represent the true position of the robot.
     */
    protected record TruePosition(double x, double y) {
    }

    /**
     * Checks if there is a valid camera to target with and if the camera has seen the tag it is trying to target.
     *
     * @param tagSet The tagSet to check against if targeting is possible.
     * @return Whether the tag is targetable.
     * @throws Error if the camera is not set before attempting to get a RobotPosition.
     */
    protected static boolean canTarget(A05Constants.AprilTagSet tagSet) {
        if (camera == null) {
            throw new Error("RobotPosition camera was not set before attempting to get a RobotPosition");
        }
        return camera.hasTargets(tagSet);
    }
}
