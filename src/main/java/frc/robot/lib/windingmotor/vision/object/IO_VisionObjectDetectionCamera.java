// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.windingmotor.vision.object;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Real hardware implementation of object detection IO using PhotonVision.
 *
 * <p>Interfaces with PhotonVision's object detection pipeline to: - Detect FUEL balls using custom
 * YOLO model - Filter detections based on confidence threshold - Track objects frame-to-frame to
 * prevent double counting - Maintain estimated count for hopper cameras
 *
 * <p>Supports two counting modes: 1. Simple mode: Just count visible objects each frame (intake
 * camera) 2. Tracking mode: Track objects entering/exiting regions (hopper camera)
 */
public class IO_VisionObjectDetectionCamera implements IO_VisionObjectDetection {
	/** PhotonVision camera instance for this IO object. */
	protected final PhotonCamera camera;

	/** Minimum confidence threshold for accepting detections [0.0, 1.0]. */
	protected final double confidenceThreshold;

	/** Whether to use tracking mode (true for hopper, false for intake). */
	protected final boolean useTracking;

	/** Entry zone Y-coordinate threshold (pixels) for hopper tracking. */
	protected final double entryZoneY;

	/** Exit zone Y-coordinate threshold (pixels) for hopper tracking. */
	protected final double exitZoneY;

	/** Current estimated count (for tracking mode). */
	protected int estimatedCount = 0;

	/** Frame counter for tracking. */
	protected long frameCount = 0;

	/** Map of tracking IDs to last known positions (for tracking mode). */
	protected Map<Integer, BallPosition> lastPositions = new HashMap<>();

	/** Set of tracking IDs that have been counted this session. */
	protected Set<Integer> countedBalls = new HashSet<>();

	/**
	 * Creates a new IO_VisionObjectDetectionCamera for a PhotonVision camera.
	 *
	 * @param name The configured name of the camera in PhotonVision
	 * @param confidenceThreshold Minimum confidence to accept detections [0.0, 1.0]
	 * @param useTracking Whether to use tracking mode for counting
	 * @param entryZoneY Y-coordinate for entry zone (pixels, -1 to disable)
	 * @param exitZoneY Y-coordinate for exit zone (pixels, -1 to disable)
	 */
	public IO_VisionObjectDetectionCamera(
			String name,
			double confidenceThreshold,
			boolean useTracking,
			double entryZoneY,
			double exitZoneY) {
		camera = new PhotonCamera(name);
		this.confidenceThreshold = confidenceThreshold;
		this.useTracking = useTracking;
		this.entryZoneY = entryZoneY;
		this.exitZoneY = exitZoneY;
	}

	/**
	 * Simplified constructor for intake camera (no tracking).
	 *
	 * @param name The configured name of the camera in PhotonVision
	 * @param confidenceThreshold Minimum confidence to accept detections [0.0, 1.0]
	 */
	public IO_VisionObjectDetectionCamera(String name, double confidenceThreshold) {
		this(name, confidenceThreshold, false, -1, -1);
	}

	@Override
	public void updateInputs(ObjectDetectionIOInputs inputs) {
		// Check if camera is connected
		inputs.connected = camera.isConnected();

		if (!inputs.connected) {
			inputs.objectCount = 0;
			inputs.detectedObjects = new DetectedObject[0];
			return;
		}

		// Process latest camera result
		var result = camera.getLatestResult();

		if (result == null || !result.hasTargets()) {
			inputs.objectCount = 0;
			inputs.detectedObjects = new DetectedObject[0];
			return;
		}

		// Update timestamp and frame count
		inputs.latestTimestamp = result.getTimestampSeconds();
		frameCount++;
		inputs.frameCount = frameCount;

		// Process all detected targets
		List<PhotonTrackedTarget> targets = result.getTargets();
		List<DetectedObject> filteredObjects = new ArrayList<>();

		// Track which balls are still visible this frame (for tracking mode)
		Set<Integer> visibleBalls = new HashSet<>();

		for (PhotonTrackedTarget target : targets) {
			// Get object detection confidence
			float confidence = target.getDetectedObjectConfidence();

			// Skip if confidence is too low or doesn't exist
			if (confidence < 0 || confidence < confidenceThreshold) {
				continue;
			}

			// Get class information
			int classId = target.getDetectedObjectClassID();
			// Map class ID to name (customize based on your model's classes)
			String className = getClassName(classId);

			// Get bounding box information from corners
			// PhotonVision provides corners, we need to calculate center and size
			var corners = target.getDetectedCorners();
			double minX = Double.MAX_VALUE, maxX = Double.MIN_VALUE;
			double minY = Double.MAX_VALUE, maxY = Double.MIN_VALUE;

			for (var corner : corners) {
				minX = Math.min(minX, corner.x);
				maxX = Math.max(maxX, corner.x);
				minY = Math.min(minY, corner.y);
				maxY = Math.max(maxY, corner.y);
			}

			double centerX = (minX + maxX) / 2.0;
			double centerY = (minY + maxY) / 2.0;
			double width = maxX - minX;
			double height = maxY - minY;

			// Use target yaw and pitch for angle information
			double yaw = target.getYaw();
			double pitch = target.getPitch();

			// Generate a simple tracking ID based on position
			// PhotonVision doesn't provide tracking IDs by default, so we create one
			int trackingId = useTracking ? generateTrackingId(centerX, centerY) : -1;

			if (useTracking) {
				visibleBalls.add(trackingId);
			}

			// Create detected object record
			DetectedObject obj =
					new DetectedObject(
							classId,
							className,
							confidence,
							centerX,
							centerY,
							width,
							height,
							yaw,
							pitch,
							trackingId);

			filteredObjects.add(obj);

			// Update tracking logic (for hopper camera)
			if (useTracking) {
				updateTracking(obj);
			}
		}

		// Clean up tracking for balls that are no longer visible
		if (useTracking) {
			cleanupTracking(visibleBalls);
		}

		// Update inputs
		inputs.objectCount = filteredObjects.size();
		inputs.detectedObjects = filteredObjects.toArray(new DetectedObject[0]);
		inputs.estimatedTotalCount = estimatedCount;
	}

	/**
	 * Updates tracking state for a detected object.
	 *
	 * <p>Detects when balls cross entry/exit zones and updates the estimated count.
	 */
	protected void updateTracking(DetectedObject obj) {
		BallPosition lastPos = lastPositions.get(obj.trackingId());
		BallPosition currentPos = new BallPosition(obj.centerX(), obj.centerY());

		if (lastPos == null) {
			// First time seeing this ball
			lastPositions.put(obj.trackingId(), currentPos);
			return;
		}

		// Check if ball crossed entry zone (entering hopper)
		if (entryZoneY > 0
				&& lastPos.y < entryZoneY
				&& currentPos.y >= entryZoneY
				&& !countedBalls.contains(obj.trackingId())) {
			// Ball entered hopper
			estimatedCount++;
			countedBalls.add(obj.trackingId());
		}

		// Check if ball crossed exit zone (leaving hopper for shooter)
		if (exitZoneY > 0 && lastPos.y < exitZoneY && currentPos.y >= exitZoneY && estimatedCount > 0) {
			// Ball exited hopper
			estimatedCount--;
			// Remove from counted set so it can be recounted if it comes back
			countedBalls.remove(obj.trackingId());
		}

		// Update position
		lastPositions.put(obj.trackingId(), currentPos);
	}

	/**
	 * Cleans up tracking data for balls that are no longer visible.
	 *
	 * @param visibleBalls Set of tracking IDs still visible this frame
	 */
	protected void cleanupTracking(Set<Integer> visibleBalls) {
		// Remove tracking data for balls not seen in the last few frames
		lastPositions.keySet().removeIf(id -> !visibleBalls.contains(id));
		// Keep countedBalls set to prevent recounting until reset
	}

	/**
	 * Generates a simple tracking ID based on position.
	 *
	 * <p>This is a simplified approach. For production, consider using a proper tracking algorithm
	 * like ByteTrack or SORT.
	 *
	 * @param x X-coordinate of object center
	 * @param y Y-coordinate of object center
	 * @return A tracking ID
	 */
	protected int generateTrackingId(double x, double y) {
		// Simple grid-based ID (divide frame into cells)
		int cellSize = 50; // pixels
		int gridX = (int) (x / cellSize);
		int gridY = (int) (y / cellSize);
		return gridY * 100 + gridX; // Simple hash
	}

	/**
	 * Maps class ID to human-readable name.
	 *
	 * <p>Customize this based on your model's class labels.
	 *
	 * @param classId The class ID from the neural network
	 * @return Human-readable class name
	 */
	protected String getClassName(int classId) {
		// Based on the Roboflow model classes
		switch (classId) {
			case 0:
				return "background";
			case 1:
				return "Fuels";
			case 2:
				return "balls";
			case 3:
				return "fuel";
			default:
				return "unknown";
		}
	}

	@Override
	public void resetCount() {
		estimatedCount = 0;
		countedBalls.clear();
		lastPositions.clear();
	}

	@Override
	public void setCount(int count) {
		estimatedCount = Math.max(0, count);
	}

	@Override
	public void decrementCount(int amount) {
		estimatedCount = Math.max(0, estimatedCount - amount);
	}

	/** Simple record to track ball positions. */
	protected static record BallPosition(double x, double y) {}
}
