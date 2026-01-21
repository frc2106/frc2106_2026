// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.windingmotor.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Base interface for gyroscope IO operations.
 *
 * <p>Provides abstraction for gyro hardware, enabling real, simulation, and replay implementations.
 * The gyro provides yaw (rotation about Z-axis) data for robot heading and odometry.
 *
 * <p>Key features: - Connection status monitoring for health alerts - High-frequency odometry data
 * via timestamped queues (250Hz on CAN FD) - Velocity data for advanced control algorithms
 *
 * <p>The IO pattern allows the same Drive subsystem to work with: - Real Pigeon 2 gyro
 * (IO_GyroReal) - Simulation gyro (IO_GyroBase empty implementation) - Replay from logs
 * (AdvantageKit auto-generated class)
 */
public interface IO_GyroBase {
	/**
	 * Container for gyro inputs that will be logged and replayed.
	 *
	 * <p>All fields are populated by IO implementations and consumed by the Drive subsystem.
	 * The @AutoLog annotation generates VisionIOInputsAutoLogged for replay capabilities.
	 */
	@AutoLog
	public static class GyroInputs {
		/** Whether the gyro is connected and responding to CAN messages. */
		public boolean connected = false;

		/** Current yaw position (rotation about Z-axis). */
		public Rotation2d yawPosition = new Rotation2d();

		/** Current yaw angular velocity in radians per second. */
		public double yawVelocityRadPerSec = 0.0;

		/**
		 * Array of timestamps for odometry samples. Used for high-frequency pose estimation (250Hz on
		 * CAN FD networks). Each timestamp corresponds to the yaw position at the same array index.
		 */
		public double[] odometryYawTimestamps = new double[] {};

		/**
		 * Array of yaw positions corresponding to odometry timestamps. These are processed by the
		 * odometry thread to update pose at high frequency.
		 */
		public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
	}

	/**
	 * Updates the inputs object with latest gyro data.
	 *
	 * <p>Implementations should: - Check connection status and set connected flag - Read current yaw
	 * position and velocity - Populate odometry queues with timestamped samples - Clear queues after
	 * reading to prevent memory growth
	 *
	 * @param inputs The inputs object to populate
	 */
	public default void updateInputs(GyroInputs inputs) {}
}
