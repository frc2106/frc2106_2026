// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.Set;

/**
 * Vision system constants for multi-camera AprilTag tracking.
 *
 * <p>This file contains all configuration for PhotonVision cameras including: - Camera names and
 * transforms (position/rotation relative to robot center) - Standard deviation models for pose
 * estimation uncertainty - Quality filtering thresholds for rejecting bad observations - Allowed
 * tag whitelist for security
 *
 * <p><b>RECONFIGURATION GUIDE:</b> Each season, update these sections in order: 1. AprilTag Field
 * Layout (new game field) 2. Camera Names (from PhotonVision web interface) 3. Camera Transforms
 * (measure carefully with CAD) 4. Filtering Thresholds (tune based on field testing) 5. Standard
 * Deviation Factors (tune based on pose estimation accuracy) 6. Allowed Tag IDs (based on best tags
 * for field)
 */
public class LIB_VisionConstants {

	// ====================================================================
	// SECTION 1: FIELD LAYOUT
	// ====================================================================

	/** AprilTag field layout for the current game. Update each season. */
	public static AprilTagFieldLayout aprilTagLayout =
			AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

	// ====================================================================
	// SECTION 2: CAMERA 0 - FRONT CAMERA
	// ====================================================================

	public static String camera0Name = "OV2311_11"; // PhotonVision camera name
	public static Transform3d robotToCamera0 =
			new Transform3d(
					new Translation3d(
							Units.inchesToMeters(12.5), // X: forward from robot center (in)
							Units.inchesToMeters(0), // Y: left from robot center (in)
							Units.inchesToMeters(7.5)), // Z: up from robot center (in)
					new Rotation3d(
							Units.degreesToRadians(0), // Roll: rotation about X-axis (deg)
							Units.degreesToRadians(-10), // Pitch: rotation about Y-axis (deg)
							Units.degreesToRadians(0))); // Yaw: rotation about Z-axis (deg)

	// ====================================================================
	// SECTION 3: CAMERA 1 - BACK CAMERA
	// ====================================================================

	public static String camera1Name = "OV2311_12"; // PhotonVision camera name
	public static Transform3d robotToCamera1 =
			new Transform3d(
					new Translation3d(
							Units.inchesToMeters(-11.25 - 2.0), // X: forward from robot center (in)
							Units.inchesToMeters(9), // Y: left from robot center (in)
							Units.inchesToMeters(19.5)), // Z: up from robot center (in)
					new Rotation3d(
							Units.degreesToRadians(180), // Roll: rotation about X-axis (deg)
							Units.degreesToRadians(-155 - 15), // Pitch: rotation about Y-axis (deg)
							0)); // Yaw: rotation about Z-axis (deg)

	// ====================================================================
	// SECTION 4: CAMERA 2 - FRONT LEFT CAMERA
	// ====================================================================

	public static String camera2Name = "OV2311_13"; // PhotonVision camera name
	public static Transform3d robotToCamera2 =
			new Transform3d(
					new Translation3d(
							Units.inchesToMeters(2.0), // X: forward from robot center (in)
							Units.inchesToMeters(9.5), // Y: left from robot center (in)
							Units.inchesToMeters(19)), // Z: up from robot center (in)
					new Rotation3d(
							0, // Roll: rotation about X-axis (deg)
							Units.degreesToRadians(-15.0), // Pitch: rotation about Y-axis (deg)
							Units.degreesToRadians(45))); // Yaw: rotation about Z-axis (deg)

	// ====================================================================
	// SECTION 5: CAMERA 3 - FRONT RIGHT CAMERA
	// ====================================================================

	public static String camera3Name = "OV2311_14"; // PhotonVision camera name
	public static Transform3d robotToCamera3 =
			new Transform3d(
					new Translation3d(
							Units.inchesToMeters(2.0), // X: forward from robot center (in)
							Units.inchesToMeters(-9.5), // Y: left from robot center (in)
							Units.inchesToMeters(19)), // Z: up from robot center (in)
					new Rotation3d(
							Units.degreesToRadians(270 + 45), // Roll: rotation about X-axis (deg)
							Units.degreesToRadians(-15.0), // Pitch: rotation about Y-axis (deg)
							Units.degreesToRadians(-45))); // Yaw: rotation about Z-axis (deg)

	// ====================================================================
	// SECTION 6: QUALITY FILTERING THRESHOLDS
	// ====================================================================

	public static double maxAmbiguity = 0.3; // Maximum pose ambiguity (0-1, lower is better)
	public static double maxZError = 0.75; // Maximum Z-axis error tolerance (m)

	// ====================================================================
	// SECTION 7: STANDARD DEVIATION MODEL (Uncertainty)
	// ====================================================================

	/**
	 * Standard deviation baselines for 1 meter distance with 1 tag.
	 *
	 * <p>Actual standard deviations scale with: - Distance² (farther = less accurate) - 1/tagCount
	 * (more tags = more accurate)
	 */
	public static double linearStdDevBaseline = 0.02; // Linear position uncertainty baseline (m)

	public static double angularStdDevBaseline = 0.06; // Angular uncertainty baseline (rad)

	/**
	 * Standard deviation multipliers for each camera.
	 *
	 * <p>Adjust these to trust some cameras more than others based on: - Mounting stability - Lens
	 * quality - Field testing results
	 */
	public static double[] cameraStdDevFactors =
			new double[] {
				1.0, // Camera 0 (Front Camera)
				1.0, // Camera 1 (Back Camera)
				1.0, // Camera 2 (Front Left Camera)
				1.0 // Camera 3 (Front Right Camera)
			};

	/**
	 * Multipliers for MegaTag 2 observations.
	 *
	 * <p>MegaTag 2 uses 2D homography instead of full 3D solve: - More stable position estimates
	 * (lower linear std dev) - No rotation data available (infinite angular std dev)
	 */
	public static double linearStdDevMegatag2Factor = 0.5; // MegaTag 2 is more stable than full 3D

	public static double angularStdDevMegatag2Factor =
			Double.POSITIVE_INFINITY; // No rotation data from MegaTag 2

	// ====================================================================
	// SECTION 8: SECURITY - ALLOWED TAG IDS
	// ====================================================================

	/**
	 * Whitelist of allowed AprilTag IDs.
	 *
	 * <p>Only these tags are processed by the vision system. Others are ignored even if detected,
	 * providing security against: - Rogue tags placed by other teams - Tags from previous games -
	 * Accidental detections outside the field
	 *
	 * <p>Update this set each season based on the game manual.
	 */
	public static Set<Integer> allowedTagIds =
			Set.of(
					6,
					7,
					8,
					9,
					10,
					11, // Blue alliance tags
					17,
					18,
					19,
					20,
					21,
					22 // Red alliance tags
					);
}
