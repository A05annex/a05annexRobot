package org.a05annex.frc.subsystems;

import org.a05annex.frc.A05Constants;
import org.a05annex.util.AngleD;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleFunction;

/**
 * A wrapper around a PhotonCamera object that provides convenient access to the latest frame and target information.
 */
public class PhotonCameraWrapper {
    private static final ArrayList<PhotonCameraWrapper> cameras = new ArrayList<>();

    /**
     * The PhotonCamera to be wrapped.
     */
    public final PhotonCamera camera;
    /**
     * The height of the target above the carpet.
     */
    private final double height;
    /**
     * The angle of the camera above the horizon
     */
    private final AngleD angle;

    // Latest frame, target, and frame with target
    private PhotonPipelineResult newestFrame = new PhotonPipelineResult();
    private List<PhotonTrackedTarget> targetList = null;
    private PhotonPipelineResult frameWithTargets = null;

    private DoubleFunction<Double> xCorrectionFunction = null;

    // Whether the latest pipeline result and latest target match
    private boolean targetsAreNew = false;

    /**
     * Creates a new PhotonCameraWrapper with the specified PhotonCamera object.
     *
     * @param camera The PhotonCamera object to wrap.
     * @param height The height of the camera above the target
     * @param angle The angle above the horizon of the camera
     */
    public PhotonCameraWrapper(@NotNull PhotonCamera camera, double height, AngleD angle) {
        this.camera = camera;
        this.height = height;
        this.angle = angle;

        cameras.add(this);
    }

    /**
     * Updates tracking data for all cameras. This only needs to be run once per
     */
    public static void updateAllTrackingData() {
        cameras.forEach(PhotonCameraWrapper::updateTrackingData);
    }

    /**
     * Updates the latest frame and target information.
     */
     private void updateTrackingData() {
         if(!camera.isConnected()) {
            targetsAreNew = false;
            return;
        }

         newestFrame = camera.getLatestResult();
        if(newestFrame == null) {
            throw new NullPointerException("Newest frame was null");
        }
        if(newestFrame.hasTargets()) {
            frameWithTargets = newestFrame;
            targetList = frameWithTargets.getTargets();
            targetsAreNew = true;
        } else {
            targetsAreNew = false;
        }
    }

    /**
     * Returns the latest pipeline result.
     *
     * @return The latest pipeline result.
     */
    public PhotonPipelineResult getNewestFrame() {
        return newestFrame;
    }

    /**
     * Returns the target with one of the specified IDs from the newest frame with targets.
     *
     * @param tagSet The AprilTagSet containing the IDs of targets to retrieve.
     * @return The target with one of the specified IDs, or null if the target was not found.
     */
    public PhotonTrackedTarget getTarget(A05Constants.AprilTagSet tagSet) {
        if(targetList == null) {
            return null;
        }

        for(PhotonTrackedTarget target : targetList) {
            for(int i = 0; i < tagSet.tagIDs().length; i++) {
                if(target.getFiducialId() == tagSet.tagIDs()[i]) {
                    return target;
                }
            }
        }
        return null;
    }

    /**
     * Filters through a frame for a specific target
     * @param frame The frame to search in
     * @param tagSet The {@link org.a05annex.frc.A05Constants.AprilTagSet} defining which target to look for
     * @return the {@link PhotonTrackedTarget} described by tagSet, or null if no tag matching the tagSet was in the frame
     */
    public static @Nullable PhotonTrackedTarget filterForTarget(@NotNull PhotonPipelineResult frame, A05Constants.AprilTagSet tagSet) {
        for(PhotonTrackedTarget target : frame.getTargets()) {
            for(int i = 0; i < tagSet.tagIDs().length; i++) {
                if(target.getFiducialId() == tagSet.tagIDs()[i]) {
                    return target;
                }
            }
        }
        return null;
    }

    /**
     * Wraps the {@link #getTarget(A05Constants.AprilTagSet)} method to return a boolean of whether the camera has seen a specific target.
     * @param tagSet The {@link A05Constants.AprilTagSet} defining which specific tag(s) to check for the visibility of.
     * @return (boolean) did the latest frame with targets contain a target specified by the tagSet passed in.
     */
    public boolean hasTargets(A05Constants.AprilTagSet tagSet) {
        return getTarget(tagSet) != null;
    }

    /**
     * Returns the timestamp of the last target seen.
     *
     * @return The timestamp of the last target seen.
     */
    public double getLatestTargetTime() {
        return frameWithTargets.getTimestampSeconds();
    }

    /**
     * Returns whether the most recent target data was from the most recent PhotonPipelineResult
     *
     * @return Whether the latest pipeline result and latest target match.
     */
    public boolean isTargetDataNew() {
        return targetsAreNew;
    }

    /**
     * Returns whether the last frame and target match the specified target IDs.
     *
     * @param tagSet The target IDs to check.
     * @return Whether the last frame and target match the specified target IDs.
     */
    public boolean isTargetDataNew(A05Constants.AprilTagSet tagSet) {
        return targetsAreNew && getTarget(tagSet) != null;
    }

    /**
     * Returns all targets detected in the latest frame.
     *
     * @return List of all targets detected in the latest frame.
     */
    public List<PhotonTrackedTarget> getLatestTargets() {
        if (frameWithTargets.hasTargets()) {
            return targetList;
        } else {
            return Collections.emptyList();
        }
    }

    /**
     * Returns the latest frame that has at least one target detected.
     *
     * @return The latest frame that has at least one target detected.
     */
    public PhotonPipelineResult getNewestFrameWithTarget() {
        return frameWithTargets;
    }


    /**
     * Returns the X coordinate (forward/backward) of the last detected target relative to the camera.
     * @param tagSet the {@link A05Constants.AprilTagSet} defining which specific tag(s) to get the X of.
     * @return the X coordinate (forward/backward) of the last detected target relative to the camera.
     */

    public double getXFromLastTarget(A05Constants.AprilTagSet tagSet) {
        if(getTarget(tagSet) == null) {
            throw new NullPointerException("A tag with the correct ID was not in the most recent frame. Make sure getTarget(AprilTagSet) does not return null before running this method");
        }

        double reportedX = getTarget(tagSet).getBestCameraToTarget().getX();

        if(xCorrectionFunction == null) {
            return reportedX;
        }

        return xCorrectionFunction.apply(reportedX);
    }

    /**
     * Returns the Y coordinate (left/right) of the last detected target relative to the camera.
     * Note that this method returns the negative Y coordinate, as the Y axis of the image
     * is inverted with respect to the Y axis of the camera coordinate system.
     * @param tagSet the {@link A05Constants.AprilTagSet} defining which specific tag(s) to get the Y of.
     * @return the Y coordinate (left/right) of the last detected target relative to the camera.
     */
    public double getYFromLastTarget(A05Constants.AprilTagSet tagSet) {
        if(getTarget(tagSet) == null) {
            throw new NullPointerException("A tag with the correct ID was not in the most recent frame. Make sure getTarget(AprilTagSet) does not return null before running this method");
        }

        return -getTarget(tagSet).getBestCameraToTarget().getY();
    }

//    /**
//     * Returns the Z coordinate of the last detected target relative to the camera.
//     * @return the Z coordinate of the last detected target relative to the camera.
//     */
//    public double getZFromLastTarget() {
//        return newestTarget.getBestCameraToTarget().getZ();
//    }
//
//    /**
//     * Calculates the horizontal angle offset in radians between the robot heading and the last detected target.
//     * The result is obtained using the atan2 function with the X and Y coordinates of the last detected target.
//     * @return The horizontal angle offset in radians between the robot heading and the last detected target.
//     */
//    public double getHorizontalOffsetRadians() {
//        return Math.atan2(getXFromLastTarget(), getYFromLastTarget());
//    }

    /**
     * Sets the X correction function. This function should take the reported X from PhotonVision as the input and
     * return what the true X (as found with a tape measure) is.
     * @param xCorrectionFunction a method that takes one double as an input, reported X, and returns what the true X
     *                            should be.
     */
    public void setXCorrectionFunction(DoubleFunction<Double> xCorrectionFunction) {
        if(this.xCorrectionFunction != null) {
            throw new IllegalStateException("You tried to set the X correction function more than once for camera: " + camera.getName());
        }
        this.xCorrectionFunction = xCorrectionFunction;
    }
}