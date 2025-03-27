package org.a05annex.frc.subsystems;

import org.a05annex.frc.A05Constants;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.*;
import java.util.function.DoubleFunction;

/**
 * A wrapper around a PhotonCamera object that provides convenient access to the latest frame and target information.
 */
public class PhotonCameraWrapper {
	private static final ArrayList<PhotonCameraWrapper> cameras = new ArrayList<>();

	/**
	 * The {@link PhotonCamera} object this class wraps
	 */
	public final PhotonCamera camera;
	/**
	 * The height of the camera, in meters, above the ground
	 */
	public final double xDisplacement, yDisplacement, headingAngle;

	// Latest frame, target, and frame with target
	private PhotonPipelineResult newestFrame = new PhotonPipelineResult();
	private List<PhotonTrackedTarget> targetList = null;
	private PhotonPipelineResult frameWithTargets = null;

	// Maps to store the latest instance of each tag ID
	private final Map<Integer, PhotonTrackedTarget> latestTargets = new HashMap<>();
	private final Map<Integer, PhotonPipelineResult> latestFrames = new HashMap<>();

	private DoubleFunction<Double> xCorrectionFunction = null;

	private DoubleFunction<Double> yCorrectionFunction = null;

	// Whether the latest pipeline result and latest target match
	private final Map<Integer, Boolean> targetsAreNew = new HashMap<>();

	/**
	 * Creates a new PhotonCameraWrapper with the specified PhotonCamera object.
	 *
	 * @param camera        The PhotonCamera object to wrap.
	 * @param xDisplacement The forward displacement of the camera, in meters, from the center of rotation
	 * @param yDisplacement The sideways displacement of the camera, in meters, from the center of rotation. Positive is to the right.
	 * @param headingAngle  The angle of the camera, in degrees, from the front of the robot. Positive is clockwise.
	 */
	@SuppressWarnings("unused")
	public PhotonCameraWrapper(@NotNull PhotonCamera camera, double xDisplacement, double yDisplacement, double headingAngle) {
		this.camera = camera;
		this.xDisplacement = xDisplacement;
		this.yDisplacement = yDisplacement;
		this.headingAngle = headingAngle;

		cameras.add(this);
	}

	/**
	 * Updates tracking data for all cameras. This only needs to be run once per cycle
	 */
	public static void updateAllTrackingData() {
		cameras.forEach(PhotonCameraWrapper::updateTrackingData);
	}

	/**
	 * Updates the latest frame and target information.
	 */
	private void updateTrackingData() {
		if(!camera.isConnected()) {
			targetsAreNew.clear();
			return;
		}

		List<PhotonPipelineResult> resultList = camera.getAllUnreadResults();

		if(resultList.isEmpty()) {
			targetsAreNew.clear();
			return;
		}

		newestFrame = resultList.get(resultList.size() - 1);

		if(newestFrame == null) {
			throw new NullPointerException("Newest frame was null");
		}

		for(PhotonPipelineResult result : resultList) {
			if(result.hasTargets()) {
				frameWithTargets = result;
				targetList = frameWithTargets.getTargets();

				// Update the latest targets and frames maps
				for(PhotonTrackedTarget target : targetList) {
					int tagId = target.getFiducialId();
					latestTargets.put(tagId, target);
					latestFrames.put(tagId, frameWithTargets);
					targetsAreNew.put(tagId, result == resultList.get(resultList.size() - 1));
				}
				break;
			}
		}
	}

	/**
	 * Returns the latest pipeline result.
	 *
	 * @return The latest pipeline result.
	 */
	@SuppressWarnings("unused")
	public PhotonPipelineResult getNewestFrame() {
		return newestFrame;
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
	 * Filters through a frame for a specific target
	 *
	 * @param frame  The frame to search in
	 * @param tagSet The {@link org.a05annex.frc.A05Constants.AprilTagSet} defining which target to look for
	 * @return the {@link PhotonTrackedTarget} described by tagSet, or null if no tag matching the tagSet was in the frame
	 */
	@Nullable
	public static PhotonTrackedTarget filterForTarget(@NotNull PhotonPipelineResult frame, A05Constants.AprilTagSet tagSet) {
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
	 * Wraps the {@link #getTarget(A05Constants.AprilTagSet)} method to return a boolean: whether the camera has seen a specific target.
	 *
	 * @param tagSet The {@link A05Constants.AprilTagSet} defining which specific tag(s) to check for the visibility of.
	 * @return (boolean) did the latest frame with targets contain a target specified by the tagSet passed in.
	 */
	public boolean hasTargets(A05Constants.AprilTagSet tagSet) {
		return getTarget(tagSet) != null;
	}

	private PhotonTrackedTarget getTarget(A05Constants.AprilTagSet tagSet) {
		for(int tagId : tagSet.tagIDs()) {
			PhotonTrackedTarget target = latestTargets.get(tagId);
			if(target != null) {
				return target;
			}
		}
		return null;
	}

	/**
	 * Returns the timestamp of the last target seen.
	 *
	 * @return The timestamp of the last target seen.
	 */
	@SuppressWarnings("unused")
	public double getLatestTargetTime() {
		return frameWithTargets.getTimestampSeconds();
	}

	/**
	 * Returns whether the most recent target data was from the most recent PhotonPipelineResult
	 *
	 * @param tagId The ID of the target to check.
	 * @return Whether the latest pipeline result and latest target match for the specified tag ID.
	 */
	@SuppressWarnings("unused")
	public boolean isTargetDataNew(int tagId) {
		return targetsAreNew.getOrDefault(tagId, false);
	}

	/**
	 * Returns whether the last frame and target match the specified target IDs.
	 *
	 * @param tagSet The target IDs to check.
	 * @return Whether the last frame and target match the specified target IDs.
	 */
	public boolean isTargetDataNew(A05Constants.AprilTagSet tagSet) {
		for(int tagId : tagSet.tagIDs()) {
			if(targetsAreNew.getOrDefault(tagId, false)) {
				return true;
			}
		}
		return false;
	}

	/**
	 * Returns all targets detected in the latest frame.
	 *
	 * @return List of all targets detected in the latest frame.
	 */
	@SuppressWarnings("unused")
	public List<PhotonTrackedTarget> getLatestTargets() {
		if(frameWithTargets.hasTargets()) {
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
	 * Returns the latest target for the specified tag ID.
	 *
	 * @param tagId The ID of the target to retrieve.
	 * @return The latest target with the specified ID, or null if the target was not found.
	 */
	@Nullable
	public PhotonTrackedTarget getLatestTarget(int tagId) {
		return latestTargets.get(tagId);
	}

	/**
	 * Returns the latest frame for the specified tag ID.
	 *
	 * @param tagId The ID of the target to retrieve the frame for.
	 * @return The latest frame with the specified target ID, or null if the target was not found.
	 */
	@Nullable
	public PhotonPipelineResult getLatestFrameForTag(int tagId) {
		return latestFrames.get(tagId);
	}

	/**
	 * Returns the X coordinate (forward/backward) of the last detected target relative to the camera.
	 *
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
	 *
	 * @param tagSet the {@link A05Constants.AprilTagSet} defining which specific tag(s) to get the Y of.
	 * @return the Y coordinate (left/right) of the last detected target relative to the camera.
	 */
	public double getYFromLastTarget(A05Constants.AprilTagSet tagSet) {
		if(getTarget(tagSet) == null) {
			throw new NullPointerException("A tag with the correct ID was not in the most recent frame. Make sure getTarget(AprilTagSet) does not return null before running this method");
		}

		double reportedY = getTarget(tagSet).getBestCameraToTarget().getY();

		if(yCorrectionFunction == null) {
			return -1.0 * reportedY;
		}

		return -1.0 * yCorrectionFunction.apply(reportedY);
	}

	/**
	 * Sets the X correction function. This function should take the reported X from PhotonVision as the input and
	 * return what the true X (as found with a tape measure) is.
	 *
	 * @param xCorrectionFunction a method that takes one double as an input, reported X, and returns what the true X
	 *                            should be.
	 */
	@SuppressWarnings("unused")
	public void setXCorrectionFunction(DoubleFunction<Double> xCorrectionFunction) {
		if(this.xCorrectionFunction != null) {
			throw new IllegalStateException("You tried to set the X correction function more than once for camera: " + camera.getName());
		}
		this.xCorrectionFunction = xCorrectionFunction;
	}

	/**
	 * Sets the Y correction function. This function should take the reported Y from PhotonVision as the input and
	 * return what the true Y (as found with a tape measure) is.
	 *
	 * @param yCorrectionFunction a method that takes one double as an input, reported Y, and returns what the true Y
	 *                            should be.
	 */
	@SuppressWarnings("unused")
	public void setYCorrectionFunction(DoubleFunction<Double> yCorrectionFunction) {
		if(this.yCorrectionFunction != null) {
			throw new IllegalStateException("You tried to set the Y correction function more than once for camera: " + camera.getName());
		}
		this.yCorrectionFunction = yCorrectionFunction;
	}
}