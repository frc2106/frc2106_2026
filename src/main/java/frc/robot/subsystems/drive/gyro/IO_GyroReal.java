// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.phoenix.PhoenixOdometryThread;

import java.util.Queue;

/**
 * Real hardware implementation of gyro IO using CTRE Pigeon 2.
 *
 * <p>Provides high-precision yaw measurements with: - 250Hz odometry updates on CAN FD networks -
 * Timestamp synchronization with motor controller samples - Queue-based data collection for minimal
 * latency - Automatic fault detection and reporting
 *
 * <p>The Pigeon 2 is configured with optimizeBusUtilization() to minimize CAN bus load while
 * maintaining high-frequency updates. Odometry data is collected via the PhoenixOdometryThread to
 * ensure perfect timestamp synchronization with swerve modules.
 */
public class IO_GyroReal implements IO_GyroBase {
	/** Pigeon 2 IMU hardware object. */
	private final Pigeon2 pigeon =
			new Pigeon2(
					TunerConstants.DrivetrainConstants.Pigeon2Id,
					TunerConstants.DrivetrainConstants.CANBusName);

	/** Status signal for yaw position (degrees). */
	private final StatusSignal<Angle> yaw = pigeon.getYaw();

	/** Status signal for yaw velocity (degrees/sec). */
	private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

	/**
	 * Queue for yaw positions from odometry thread. Populated at ODOMETRY_FREQUENCY (250Hz on CAN FD)
	 * by PhoenixOdometryThread.
	 */
	private final Queue<Double> yawPositionQueue;

	/**
	 * Queue for timestamps from odometry thread. Each timestamp corresponds to the yaw position at
	 * the same queue index.
	 */
	private final Queue<Double> yawTimestampQueue;

	/**
	 * Constructs and configures the Pigeon 2 gyro.
	 *
	 * <p>Configuration includes: - Applying default Pigeon2Configuration for optimal settings -
	 * Zeroing the yaw angle (robot should be pointed away from drivers when powered on) - Setting
	 * update frequencies (250Hz for odometry, 50Hz for velocity) - Optimizing CAN bus utilization to
	 * minimize bandwidth - Registering with odometry thread for synchronized sampling
	 */
	public IO_GyroReal() {
		// Apply default configuration
		pigeon.getConfigurator().apply(new Pigeon2Configuration());

		// Zero yaw (robot should be pointed away from drivers when powered on)
		pigeon.getConfigurator().setYaw(0.0);

		// Set update frequencies
		// Odometry yaw at high frequency (250Hz on CAN FD)
		yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
		// Velocity at standard frequency (50Hz)
		yawVelocity.setUpdateFrequency(50.0);

		// Minimize CAN bus bandwidth usage
		pigeon.optimizeBusUtilization();

		// Register with odometry thread for timestamp-synchronized sampling
		yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
		yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
	}

	/**
	 * Updates the inputs object with latest Pigeon 2 data.
	 *
	 * <p>Performs: 1. Bulk refresh of all status signals (efficient CAN bus usage) 2. Extracts yaw
	 * position and velocity 3. Drains odometry queues into arrays for Drive subsystem processing 4.
	 * Clears queues to prevent unbounded memory growth
	 *
	 * <p>The odometry queues contain timestamped samples collected by the odometry thread. These must
	 * be drained every cycle to ensure real-time processing and prevent memory leaks.
	 */
	@Override
	public void updateInputs(GyroInputs inputs) {
		// Bulk refresh all signals (more efficient than individual calls)
		inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);

		// Convert yaw from Phoenix degrees to Rotation2d
		inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());

		// Convert yaw velocity from degrees/sec to radians/sec
		inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

		// Drain timestamp queue into array
		inputs.odometryYawTimestamps =
				yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();

		// Drain position queue into Rotation2d array
		inputs.odometryYawPositions =
				yawPositionQueue.stream()
						.map((Double value) -> Rotation2d.fromDegrees(value))
						.toArray(Rotation2d[]::new);

		// Clear queues to prevent memory growth
		// These queues accumulate samples at 250Hz, so must be drained every cycle
		yawTimestampQueue.clear();
		yawPositionQueue.clear();
	}
}
