// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.windingmotor.vision.object;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for object detection vision operations (ball counting).
 *
 * <p>Extends the base vision system to support object detection for game piece counting.
 * Specifically designed for FUEL ball detection in FRC 2026 REBOUND game.
 *
 * <p>Key capabilities: - Detection and counting of game pieces (FUEL balls) - Per-object confidence
 * scores for filtering - Bounding box data for tracking ball positions - Frame-to-frame tracking to
 * prevent double counting
 *
 * <p>This abstraction allows the same subsystem to work with real PhotonVision object detection,
 * simulation, or replayed data from logs.
 */
public interface IO_VisionObjectDetection {
	/**
	 * Container for all object detection inputs that will be logged and replayed.
	 *
	 * <p>Fields are populated by IO implementations and consumed by the object detection subsystem.
	 * The AutoLog annotation generates the toLog/fromLog methods automatically.
	 */
	@AutoLog
	public static class ObjectDetectionIOInputs {
		/** Whether the camera is currently connected and responding. */
		public boolean connected = false;

		/** Timestamp of the latest frame processed (seconds). */
		public double latestTimestamp = 0.0;

		/** Total number of objects detected in the latest frame. */
		public int objectCount = 0;

		/** Array of all detected objects in the latest frame. */
		public DetectedObject[] detectedObjects = new DetectedObject[0];

		/** Latest estimated total count (for hopper with tracking). */
		public int estimatedTotalCount = 0;

		/** Frame counter for tracking purposes. */
		public long frameCount = 0;
	}

	/**
	 * Represents a single detected object (FUEL ball) with position and confidence.
	 *
	 * <p>Contains all information needed for filtering and tracking: - Class ID and name for
	 * identifying object type - Confidence score for quality filtering - Bounding box for position
	 * and size - Unique ID for frame-to-frame tracking (if available)
	 */
	public static record DetectedObject(
			int classId, // Object class ID from neural network
			String className, // Human-readable class name ("Fuels", "balls", etc.)
			double confidence, // Detection confidence [0.0, 1.0]
			double centerX, // Bounding box center X (pixels)
			double centerY, // Bounding box center Y (pixels)
			double width, // Bounding box width (pixels)
			double height, // Bounding box height (pixels)
			double yaw, // Horizontal angle to object (degrees)
			double pitch, // Vertical angle to object (degrees)
			int trackingId // Unique ID for this object (-1 if not tracked)
			) {}

	/**
	 * Updates the inputs object with latest object detection data from the camera.
	 *
	 * <p>Implementations should populate all fields of the inputs object, including: - Connection
	 * status - Frame timestamp and count - All detected objects with confidence above threshold -
	 * Estimated total count (for tracking-based implementations)
	 *
	 * @param inputs The inputs object to populate
	 */
	public default void updateInputs(ObjectDetectionIOInputs inputs) {}

	/**
	 * Resets the internal counting state (for hopper cameras).
	 *
	 * <p>Used when the hopper is known to be empty or when the count needs manual correction.
	 */
	public default void resetCount() {}

	/**
	 * Manually sets the estimated count (for hopper cameras).
	 *
	 * <p>Useful for correcting drift or initializing to a known state.
	 *
	 * @param count The count to set
	 */
	public default void setCount(int count) {}

	/**
	 * Decrements the estimated count (called when balls are shot).
	 *
	 * <p>For hopper cameras with tracking-based counting.
	 *
	 * @param amount Number of balls to subtract
	 */
	public default void decrementCount(int amount) {}
}
