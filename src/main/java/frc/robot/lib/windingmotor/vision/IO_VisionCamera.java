// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.windingmotor.vision;

import static frc.robot.constants.LIB_VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/**
 * Real hardware implementation of vision IO using PhotonVision cameras.
 *
 * <p>Interfaces with PhotonVision's PhotonCamera API to: - Read multitag and single-tag pose
 * estimation results - Extract target observations for servoing - Calculate robot poses from tag
 * detections - Track all detected tag IDs for debugging
 *
 * <p>Handles both multitag results (more accurate, uses multiple tags) and single-tag results (less
 * accurate but works when only one tag is visible). All observations are timestamped for correct
 * fusion with the pose estimator.
 */
public class IO_VisionCamera implements IO_VisionBase {
	/** PhotonVision camera instance for this IO object. */
	protected final PhotonCamera camera;

	/**
	 * Transform from robot center to camera mounting position. Used to convert camera poses to robot
	 * poses.
	 */
	protected final Transform3d robotToCamera;

	/**
	 * Creates a new IO_VisionCamera for a PhotonVision camera.
	 *
	 * @param name The configured name of the camera in PhotonVision
	 * @param robotToCamera The 3D transform from robot center to camera mounting position
	 */
	public IO_VisionCamera(String name, Transform3d robotToCamera) {
		camera = new PhotonCamera(name);
		this.robotToCamera = robotToCamera;
	}

	/**
	 * Updates the inputs object with latest data from the PhotonVision camera.
	 *
	 * <p>Processes all unread results from the camera, extracting: - Connection status - Best target
	 * observation (for servoing) - All pose observations (multitag and single-tag) - All detected tag
	 * IDs
	 *
	 * <p>Multitag results are more accurate as they use multiple tags simultaneously. Single-tag
	 * results are used as fallback when only one tag is visible. All poses are converted from
	 * camera-relative to robot-relative coordinates using the robotToCamera transform.
	 */
	@Override
	public void updateInputs(VisionIOInputs inputs) {
		// Check if camera is connected and responding
		inputs.connected = camera.isConnected();

		// Track detected tag IDs across all results
		Set<Short> tagIds = new HashSet<>();
		// Collect all pose observations from this update cycle
		List<PoseObservation> poseObservations = new LinkedList<>();

		// Process all unread camera results
		for (var result : camera.getAllUnreadResults()) {
			// Update latest target observation for servoing
			if (result.hasTargets()) {
				inputs.latestTargetObservation =
						new TargetObservation(
								Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
								Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
			} else {
				inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
			}

			// Process multitag result (most accurate, uses multiple tags)
			if (result.multitagResult.isPresent()) {
				var multitagResult = result.multitagResult.get();

				// Convert camera pose to robot pose
				// Camera pose is relative to field origin; add inverse of robot-to-camera offset
				Transform3d fieldToCamera = multitagResult.estimatedPose.best;
				Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
				Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

				// Calculate average distance to all tags in this observation
				// Used by the subsystem for standard deviation calculation
				double totalTagDistance = 0.0;
				for (var target : result.targets) {
					totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
				}

				// Record all tag IDs used in this observation
				tagIds.addAll(multitagResult.fiducialIDsUsed);

				// Add the pose observation to our collection
				poseObservations.add(
						new PoseObservation(
								result.getTimestampSeconds(), // Timestamp for pose interpolation
								robotPose, // Robot pose in field coordinates
								multitagResult.estimatedPose.ambiguity, // Pose confidence (lower is better)
								multitagResult.fiducialIDsUsed.size(), // Number of tags used
								totalTagDistance / result.targets.size(), // Average tag distance
								PoseObservationType.PHOTONVISION)); // Observation type

			} else if (!result.targets.isEmpty()) {
				// Single-tag result fallback (less accurate but still useful)
				var target = result.targets.get(0);

				// Calculate robot pose from single tag
				var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
				if (tagPose.isPresent()) {
					// Transform from field to tag, then tag to camera, then camera to robot
					Transform3d fieldToTarget =
							new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
					Transform3d cameraToTarget = target.bestCameraToTarget;
					Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
					Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
					Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

					// Record the tag ID
					tagIds.add((short) target.fiducialId);

					// Add the pose observation
					poseObservations.add(
							new PoseObservation(
									result.getTimestampSeconds(),
									robotPose,
									target.poseAmbiguity,
									1, // Single tag count
									cameraToTarget.getTranslation().getNorm(),
									PoseObservationType.PHOTONVISION));
				}
			}
		}

		// Convert LinkedList to array for the inputs object
		inputs.poseObservations = new PoseObservation[poseObservations.size()];
		for (int i = 0; i < poseObservations.size(); i++) {
			inputs.poseObservations[i] = poseObservations.get(i);
		}

		// Convert Set<Short> to int[] for logging
		inputs.tagIds = new int[tagIds.size()];
		int i = 0;
		for (int id : tagIds) {
			inputs.tagIds[i++] = id;
		}
	}
}
