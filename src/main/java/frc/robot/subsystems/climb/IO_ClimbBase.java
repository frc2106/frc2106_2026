// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Base interface for climb subsystem IO operations. Provides the contract for monitoring motor
 * telemetry and commanding motor output for the climb mechanism. Implementations may target real
 * hardware or simulation while exposing an identical API for commands.
 */
public interface IO_ClimbBase {

	/**
	 * Telemetry container for the climb motor. The fields here are logged and replayed via the
	 * logging framework to support analysis and debugging.
	 */
	@AutoLog
	public static class ClimbInputs implements LoggableInputs {

		/** Motor supply/output current in amps. */
		public double motorCurrent = 0.0;

		/** Motor position in native rotations (relative encoder unless otherwise configured). */
		public double motorPosition = 0.0;

		/** Motor velocity in rotations per minute. */
		public double motorVelocity = 0.0;

		/**
		 * Writes all telemetry to the logging table.
		 *
		 * @param table the logging table to populate
		 */
		@Override
		public void toLog(LogTable table) {
			table.put("MotorCurrent", motorCurrent);
			table.put("MotorPosition", motorPosition);
			table.put("MotorVelocity", motorVelocity);
		}

		/**
		 * Reads all telemetry from the logging table during replay.
		 *
		 * @param table the logging table to read from
		 */
		@Override
		public void fromLog(LogTable table) {
			motorCurrent = table.get("MotorCurrent", motorCurrent);
			motorPosition = table.get("MotorPosition", motorPosition);
			motorVelocity = table.get("MotorVelocity", motorVelocity);
		}
	}

	/**
	 * Refreshes climb telemetry into the provided inputs container. Call once per loop to keep logs
	 * and dashboards up to date.
	 *
	 * @param inputs structure to populate with the latest sensor and status values
	 */
	public void updateInputs(ClimbInputs inputs);

	/**
	 * Sets the climb motor output as a duty cycle setpoint. Positive values extend or retract
	 * depending on wiring and inversion configuration.
	 *
	 * @param speed duty cycle command in the range [-1.0, 1.0]
	 */
	public void setMotorSpeed(double speed);
}
