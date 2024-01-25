package org.a05annex.frc.subsystems;

import org.jetbrains.annotations.Nullable;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Collections;
import java.util.List;

/**
 * A wrapper around a PhotonCamera object that provides convenient access to the latest frame and target information.
 */
public class PhotonCameraWrapper {

    private final PhotonCamera camera;

    // Latest frame, target, and frame with target
    private PhotonPipelineResult latestFrame = new PhotonPipelineResult();
    private PhotonTrackedTarget latestTarget = null;
    private PhotonPipelineResult latestFrameWithTarget = new PhotonPipelineResult();

    // ID and time of last target seen
    private double lastTargetId = -1;

    // Whether the latest pipeline result and latest target match
    private boolean latestPipelineResultAndLatestTargetMatch;

    /**
     * Creates a new PhotonCameraWrapper with the specified PhotonCamera object.
     *
     * @param camera The PhotonCamera object to wrap.
     */
    public PhotonCameraWrapper(PhotonCamera camera) {
        if(camera == null) {
            throw new IllegalArgumentException("PhotonCamera object cannot be null.");
        }
        this.camera = camera;
    }


    /**
     * Updates the latest frame and target information.
     */
    public void updateLatestFrameAndTarget() {
        latestFrame = camera.getLatestResult();
        if(latestFrame == null) {
            throw new NullPointerException("Latest frame or target data is null.");
        }
        if(latestFrame.hasTargets()) {
            latestFrameWithTarget = latestFrame;
            latestTarget = latestFrameWithTarget.getBestTarget();
            if(lastTargetId == latestTarget.getFiducialId()) {
                // The same target was seen again, so the latest pipeline result and latest target match
                latestPipelineResultAndLatestTargetMatch = true;
            } else {
                // A different target is now the best target
                lastTargetId = latestTarget.getFiducialId();
            }
        } else {
            latestPipelineResultAndLatestTargetMatch = false;
        }
    }


    /**
     * Returns the latest pipeline result.
     *
     * @return The latest pipeline result.
     */
    public PhotonPipelineResult getLatestFrame() {
        return latestFrame;
    }

    /**
     * Returns the latest tracked target.
     *
     * @return The latest tracked target. {@code null} if there is no latest tracked target.
     */
    @Nullable
    public PhotonTrackedTarget getLatestTarget() {
        return latestTarget;
    }

    /**
     * Returns the timestamp of the last target seen.
     *
     * @return The timestamp of the last target seen.
     */
    public double getLatestTargetTime() {
        return latestFrameWithTarget.getTimestampSeconds();
    }

    /**
     * Returns whether the latest pipeline result and latest target match.
     *
     * @return Whether the latest pipeline result and latest target match.
     */
    public boolean doesLatestFrameAndTargetMatch() {
        return latestPipelineResultAndLatestTargetMatch;
    }

    /**
     * Returns whether the last frame and target match the specified target ID.
     *
     * @param targetId The target ID to check.
     * @return Whether the last frame and target match the specified target ID.
     */
    public boolean doesLatestFrameAndTargetMatch(int targetId) {
        return latestFrame.hasTargets() && (lastTargetId == targetId);
    }

    /**
     * Returns all targets detected in the latest frame.
     *
     * @return All targets detected in the latest frame.
     */
    public List<PhotonTrackedTarget> getLatestTargets() {
        if (latestFrameWithTarget.hasTargets()) {
            return latestFrameWithTarget.getTargets();
        } else {
            return Collections.emptyList();
        }
    }

    /**
     * Returns the latest frame that has at least one target detected.
     *
     * @return The latest frame that has at least one target detected.
     */
    public PhotonPipelineResult getLatestFrameWithTarget() {
        return latestFrameWithTarget;
    }

    /**
     * Returns the target with the specified ID from the latest frame.
     *
     * @param id The ID of the target to retrieve.
     * @return The target with the specified ID from the latest frame, or null if the target was not found.
     */
    public PhotonTrackedTarget getTargetById(int id) {
        for (PhotonTrackedTarget target : getLatestTargets()) {
            if (target.getFiducialId() == id) {
                return target;
            }
        }
        return null;
    }

    /**
     * Returns whether the target with the specified ID is currently visible in the camera frame.
     *
     * @param id The ID of the target to check.
     * @return Whether the target with the specified ID is currently visible in the camera frame.
     */
    public boolean isTargetVisible(int id) {
        PhotonTrackedTarget target = getTargetById(id);
        return target != null;
    }

    /**
     * Returns the distance from the camera to the last detected target.
     *
     * @return The distance from the camera to the last detected target, in meters.
     */
    public Double getDistanceFromLastTarget() {
        return (null == latestTarget) ? null : latestTarget.getBestCameraToTarget().getTranslation().getNorm();
    }



    /**
     * Returns the X coordinate of the last detected target relative to the camera.
     * @return the X coordinate of the last detected target relative to the camera.
     */
    public Double getXFromLastTarget() {
        return (null == latestTarget) ? null : latestTarget.getBestCameraToTarget().getX();
    }

    /**
     * Returns the Y coordinate of the last detected target relative to the camera.
     * Note that this method returns the negative Y coordinate, as the Y axis of the image
     * is inverted with respect to the Y axis of the camera coordinate system.
     * @return the Y coordinate of the last detected target relative to the camera.
     */
    public Double getYFromLastTarget() {
        return (null == latestTarget) ? null : -latestTarget.getBestCameraToTarget().getY();
    }

    /**
     * Returns the Z coordinate of the last detected target relative to the camera.
     * @return the Z coordinate of the last detected target relative to the camera.
     */
    public Double getZFromLastTarget() {
        return (null == latestTarget) ? null : latestTarget.getBestCameraToTarget().getZ();
    }

    /**
     * Calculates the horizontal angle offset in radians between the robot heading and the last detected target.
     * The result is obtained using the atan2 function with the X and Y coordinates of the last detected target.
     * @return The horizontal angle offset in radians between the robot heading and the last detected target.
     */
    public Double getHorizontalOffsetRadians() {
        return  (null == latestTarget) ? null : Math.atan2(getXFromLastTarget(), getYFromLastTarget());
    }
}