package org.a05annex.frc.subsystems;

import org.a05annex.frc.A05Constants;
import org.a05annex.util.AngleD;
import org.jetbrains.annotations.NotNull;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Collections;
import java.util.List;

/**
 * A wrapper around a PhotonCamera object that provides convenient access to the latest frame and target information.
 */
public class PhotonCameraWrapper {

    public final PhotonCamera camera;
    private final double height;
    private final AngleD angle;

    // Latest frame, target, and frame with target
    private PhotonPipelineResult newestFrame = new PhotonPipelineResult();
    private List<PhotonTrackedTarget> targetList = null;
    private PhotonPipelineResult frameWithTargets = null;

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
    }

    /**
     * Updates the latest frame and target information.
     */
    public void updateTrackingData() {
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
        return filterForTarget(tagSet);
    }

    /**
     * Get the PhotonTrackedTarget with a matching ID from the id set contained in an AprilTagSet
     *
     * @param tagSet the AprilTagSet containing the IDs you want
     * @return a PhotonTrackedTarget matching the IDs in tagSet or null if the correct target was not present
     */
    private PhotonTrackedTarget filterForTarget(A05Constants.AprilTagSet tagSet) {
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
     * @return the X coordinate (forward/backward) of the last detected target relative to the camera.
     */

    public double getXFromLastTarget(A05Constants.AprilTagSet tagSet) {
        if(filterForTarget(tagSet) == null) {
            throw new NullPointerException("A tag with the correct ID was not in the most recent frame. Make sure getTarget(AprilTagSet) does not return null before running this method");
        }

        return filterForTarget(tagSet).getBestCameraToTarget().getX();
    }

    /**
     * Returns the Y coordinate (left/right) of the last detected target relative to the camera.
     * Note that this method returns the negative Y coordinate, as the Y axis of the image
     * is inverted with respect to the Y axis of the camera coordinate system.
     * @return the Y coordinate (left/right) of the last detected target relative to the camera.
     */
    public double getYFromLastTarget(A05Constants.AprilTagSet tagSet) {
        if(filterForTarget(tagSet) == null) {
            throw new NullPointerException("A tag with the correct ID was not in the most recent frame. Make sure getTarget(AprilTagSet) does not return null before running this method");
        }

        return -filterForTarget(tagSet).getBestCameraToTarget().getY();
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


}