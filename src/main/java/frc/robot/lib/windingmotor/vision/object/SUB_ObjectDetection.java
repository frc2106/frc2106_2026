// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.windingmotor.vision.object;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.windingmotor.vision.object.IO_VisionObjectDetection.DetectedObject;
import java.util.function.IntConsumer;
import org.littletonrobotics.junction.Logger;

/**
 * Object detection subsystem for FUEL ball counting.
 *
 * <p>Manages multiple object detection cameras (intake and hopper) and provides: - Real-time ball
 * counting for intake camera - Accumulated ball counting for hopper camera with tracking - Quality
 * filtering based on confidence thresholds - Comprehensive telemetry logging for debugging
 *
 * <p>Design philosophy: - Intake camera: Simple counting of visible balls (servoing target) -
 * Hopper camera: Tracking-based counting with entry/exit zones - Both cameras log extensively for
 * replay and tuning
 *
 * <p>Integration with robot code: - Call getIntakeBallCount() for intake ball visibility - Call
 * getHopperBallCount() for total balls in hopper - Call decrementHopperCount() when shooting balls
 * - Call resetHopperCount() when hopper is known to be empty
 */
public class SUB_ObjectDetection extends SubsystemBase {
	/** Intake camera IO implementation. */
	private final IO_VisionObjectDetection intakeIO;

	/** Hopper camera IO implementation. */
	private final IO_VisionObjectDetection hopperIO;

	/** Intake camera inputs (auto-logged). */
	private final ObjectDetectionIOInputsAutoLogged intakeInputs;

	/** Hopper camera inputs (auto-logged). */
	private final ObjectDetectionIOInputsAutoLogged hopperInputs;

	/** Alert for disconnected intake camera. */
	private final Alert intakeDisconnectedAlert;

	/** Alert for disconnected hopper camera. */
	private final Alert hopperDisconnectedAlert;

	/** Alert for hopper approaching capacity. */
	private final Alert hopperNearFullAlert;

	/** Maximum hopper capacity. */
	private static final int MAX_HOPPER_CAPACITY = 50;

	/** Threshold for "near full" alert. */
	private static final int NEAR_FULL_THRESHOLD = 45;

	/** Optional callback when hopper count changes. */
	private IntConsumer hopperCountChangeCallback = null;

	/**
	 * Constructs the object detection subsystem with intake and hopper cameras.
	 *
	 * @param intakeIO IO implementation for intake camera
	 * @param hopperIO IO implementation for hopper camera
	 */
	public SUB_ObjectDetection(IO_VisionObjectDetection intakeIO, IO_VisionObjectDetection hopperIO) {
		this.intakeIO = intakeIO;
		this.hopperIO = hopperIO;

		// Initialize auto-logged inputs
		this.intakeInputs = new ObjectDetectionIOInputsAutoLogged();
		this.hopperInputs = new ObjectDetectionIOInputsAutoLogged();

		// Initialize alerts
		this.intakeDisconnectedAlert = new Alert("Intake camera is disconnected.", AlertType.kWarning);
		this.hopperDisconnectedAlert = new Alert("Hopper camera is disconnected.", AlertType.kWarning);
		this.hopperNearFullAlert =
				new Alert(
						"Hopper is near full capacity ("
								+ NEAR_FULL_THRESHOLD
								+ "/"
								+ MAX_HOPPER_CAPACITY
								+ " balls).",
						AlertType.kInfo);
	}

	/**
	 * Gets the number of balls visible to the intake camera.
	 *
	 * <p>This is a real-time count of visible balls, useful for: - Detecting when balls are
	 * approaching - Vision-based intake servoing - Driver feedback
	 *
	 * @return Number of balls currently visible
	 */
	public int getIntakeBallCount() {
		return intakeInputs.objectCount;
	}

	/**
	 * Gets the estimated total number of balls in the hopper.
	 *
	 * <p>This is a tracked/accumulated count, not just visible balls.
	 *
	 * @return Estimated number of balls in hopper
	 */
	public int getHopperBallCount() {
		return hopperInputs.estimatedTotalCount;
	}

	/**
	 * Checks if the hopper is near full capacity.
	 *
	 * @return True if hopper count >= near-full threshold
	 */
	public boolean isHopperNearFull() {
		return hopperInputs.estimatedTotalCount >= NEAR_FULL_THRESHOLD;
	}

	/**
	 * Checks if the hopper is at or above maximum capacity.
	 *
	 * @return True if hopper count >= max capacity
	 */
	public boolean isHopperFull() {
		return hopperInputs.estimatedTotalCount >= MAX_HOPPER_CAPACITY;
	}

	/**
	 * Resets the hopper ball count to zero.
	 *
	 * <p>Call this when the hopper is known to be empty.
	 */
	public void resetHopperCount() {
		hopperIO.resetCount();
	}

	/**
	 * Manually sets the hopper ball count.
	 *
	 * <p>Useful for correcting drift or initializing to a known state.
	 *
	 * @param count The count to set
	 */
	public void setHopperCount(int count) {
		hopperIO.setCount(count);
	}

	/**
	 * Decrements the hopper count by the specified amount.
	 *
	 * <p>Call this when balls are shot from the hopper.
	 *
	 * @param amount Number of balls to subtract
	 */
	public void decrementHopperCount(int amount) {
		int previousCount = hopperInputs.estimatedTotalCount;
		hopperIO.decrementCount(amount);

		// Notify callback if count changed
		if (hopperCountChangeCallback != null && previousCount != hopperInputs.estimatedTotalCount) {
			hopperCountChangeCallback.accept(hopperInputs.estimatedTotalCount);
		}
	}

	/**
	 * Decrements the hopper count by one.
	 *
	 * <p>Convenience method for shooting a single ball.
	 */
	public void decrementHopperCount() {
		decrementHopperCount(1);
	}

	/**
	 * Sets a callback to be invoked when the hopper count changes.
	 *
	 * @param callback Consumer that receives the new count
	 */
	public void setHopperCountChangeCallback(IntConsumer callback) {
		this.hopperCountChangeCallback = callback;
	}

	/**
	 * Gets the array of detected objects from the intake camera.
	 *
	 * <p>Useful for advanced processing or visualization.
	 *
	 * @return Array of detected objects
	 */
	public DetectedObject[] getIntakeDetectedObjects() {
		return intakeInputs.detectedObjects;
	}

	/**
	 * Gets the array of detected objects from the hopper camera.
	 *
	 * <p>Useful for advanced processing or visualization.
	 *
	 * @return Array of detected objects
	 */
	public DetectedObject[] getHopperDetectedObjects() {
		return hopperInputs.detectedObjects;
	}

	/**
	 * Main periodic method that processes all camera data.
	 *
	 * <p>For each camera: 1. Updates inputs from IO layer 2. Processes connection status 3. Logs
	 * comprehensive telemetry 4. Updates alerts
	 */
	@Override
	public void periodic() {
		// Track previous hopper count for change detection
		int previousHopperCount = hopperInputs.estimatedTotalCount;

		// Update inputs from both cameras
		intakeIO.updateInputs(intakeInputs);
		hopperIO.updateInputs(hopperInputs);

		// Process inputs through AdvantageKit logging
		Logger.processInputs("ObjectDetection/Intake", intakeInputs);
		Logger.processInputs("ObjectDetection/Hopper", hopperInputs);

		// Update connection alerts
		intakeDisconnectedAlert.set(!intakeInputs.connected);
		hopperDisconnectedAlert.set(!hopperInputs.connected);

		// Update hopper capacity alert
		hopperNearFullAlert.set(isHopperNearFull());

		// Log high-level counts for dashboard
		Logger.recordOutput("ObjectDetection/IntakeCount", intakeInputs.objectCount);
		Logger.recordOutput("ObjectDetection/HopperCount", hopperInputs.estimatedTotalCount);
		Logger.recordOutput("ObjectDetection/HopperNearFull", isHopperNearFull());
		Logger.recordOutput("ObjectDetection/HopperFull", isHopperFull());

		// Log individual detected objects for visualization
		logDetectedObjects("Intake", intakeInputs.detectedObjects);
		logDetectedObjects("Hopper", hopperInputs.detectedObjects);

		// Notify callback if hopper count changed
		if (hopperCountChangeCallback != null
				&& previousHopperCount != hopperInputs.estimatedTotalCount) {
			hopperCountChangeCallback.accept(hopperInputs.estimatedTotalCount);
		}
	}

	/**
	 * Logs detailed information about detected objects for visualization.
	 *
	 * @param cameraName Name of the camera ("Intake" or "Hopper")
	 * @param objects Array of detected objects
	 */
	private void logDetectedObjects(String cameraName, DetectedObject[] objects) {
		// Log bounding box centers for 2D visualization
		double[] centersX = new double[objects.length];
		double[] centersY = new double[objects.length];
		double[] confidences = new double[objects.length];
		String[] classNames = new String[objects.length];

		for (int i = 0; i < objects.length; i++) {
			centersX[i] = objects[i].centerX();
			centersY[i] = objects[i].centerY();
			confidences[i] = objects[i].confidence();
			classNames[i] = objects[i].className();
		}

		Logger.recordOutput("ObjectDetection/" + cameraName + "/CentersX", centersX);
		Logger.recordOutput("ObjectDetection/" + cameraName + "/CentersY", centersY);
		Logger.recordOutput("ObjectDetection/" + cameraName + "/Confidences", confidences);
		Logger.recordOutput("ObjectDetection/" + cameraName + "/ClassNames", classNames);
	}
}
