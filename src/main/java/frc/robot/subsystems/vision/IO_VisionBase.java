// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Base interface for vision IO operations.
 *
 * <p>Defines the contract for reading vision data from cameras, including: - Target observations
 * (simple angle measurements for servoing) - Pose observations (full 3D robot pose estimates from
 * AprilTags) - Connection status for health monitoring
 *
 * <p>AdvantageKit's @AutoLog annotation automatically generates the VisionIOInputs class and the
 * VisionIOInputsAutoLogged subclass used for replay logging.
 *
 * <p>This abstraction allows the same subsystem to work with real PhotonVision cameras, simulation
 * implementations, or replayed data from logs.
 */
public interface IO_VisionBase {
	/**
	 * Container for all vision inputs that will be logged and replayed.
	 *
	 * <p>Fields are populated by IO implementations and consumed by the vision subsystem. The AutoLog
	 * annotation generates the toLog/fromLog methods automatically.
	 */
	@AutoLog
	public static class VisionIOInputs {
		/** Whether the camera is currently connected and responding. */
		public boolean connected = false;

		/** Latest target observation (angle to best target). Used for simple vision-based aiming. */
		public TargetObservation latestTargetObservation =
				new TargetObservation(new Rotation2d(), new Rotation2d());

		/**
		 * Array of pose observations from AprilTag detection. Each represents a robot pose estimate.
		 */
		public PoseObservation[] poseObservations = new PoseObservation[0];

		/** IDs of all tags detected in the latest frame. Used for debugging and filtering. */
		public int[] tagIds = new int[0];
	}

	/**
	 * Simple target observation containing horizontal and vertical angles to the best target.
	 *
	 * <p>Used for servoing tasks where full pose estimation isn't needed, such as aiming at a target.
	 */
	public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

	/**
	 * Full 3D pose observation with quality metrics.
	 *
	 * <p>Contains everything needed for pose estimation: the pose itself, timestamp, ambiguity (pose
	 * confidence), number of tags, average tag distance, and observation type. These metrics allow
	 * the subsystem to filter out unreliable observations.
	 */
	public static record PoseObservation(
			double timestamp,
			Pose3d pose,
			double ambiguity,
			int tagCount,
			double averageTagDistance,
			PoseObservationType type) {}

	/**
	 * Type of pose observation, indicating which algorithm produced it.
	 *
	 * <p>MEGATAG_1 and MEGATAG_2 are Limelight-specific multitag algorithms. PHOTONVISION is used for
	 * PhotonVision cameras. The type affects standard deviation scaling in the subsystem.
	 */
	public static enum PoseObservationType {
		MEGATAG_1,
		MEGATAG_2,
		PHOTONVISION
	}

	/**
	 * Updates the inputs object with latest data from the camera.
	 *
	 * <p>Implementations should populate all fields of the inputs object, including: - Connection
	 * status - Latest target observation (if any targets are visible) - All pose observations from
	 * the latest camera frame - All tag IDs detected
	 *
	 * @param inputs The inputs object to populate
	 */
	public default void updateInputs(VisionIOInputs inputs) {}
}
