// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Base interface for intake subsystem IO operations. Defines the contract for controlling and
 * monitoring the intake mechanism, including arm positioning, wheel speed control, and sensor
 * feedback. This interface supports both real hardware and simulation implementations.
 *
 * @version 1.0
 * @since 2024-2025 Season
 */
public interface IO_IntakeBase {

	/**
	 * Input data class for intake subsystem telemetry and state. Automatically logged using
	 * AdvantageKit's @AutoLog annotation. Contains all sensor readings and motor feedback for the
	 * intake mechanism.
	 */
	@AutoLog
	public static class IntakeInputs implements LoggableInputs {

		/** Current angle of the intake arm in degrees. Defaults to 0.0. */
		public double armAngleDegrees = 0.0;

		/** Current draw of the arm motor in amps. Defaults to 0.0. */
		public double armMotorCurrent = 0.0;

		/** Current draw of the intake wheels motor in amps. Defaults to 0.0. */
		public double wheelMotorCurrent = 0.0;

		/** Current RPM of the intake wheels. Defaults to 0.0. */
		public double wheelRPM = 0.0;

		/**
		 * State of the intake sensor (true = object detected, false = no object). Defaults to false.
		 */
		public boolean sensor = false;

		/**
		 * Writes all input values to the logging table for telemetry. Called automatically by
		 * AdvantageKit's logging framework.
		 *
		 * @param table The LogTable to write values to
		 */
		@Override
		public void toLog(LogTable table) {
			table.put("ArmAngleDegrees", armAngleDegrees);
			table.put("ArmMotorCurrent", armMotorCurrent);
			table.put("wheelMotorCurrent", wheelMotorCurrent);
			table.put("wheelRPM", wheelRPM);
			table.put("Sensor", sensor);
		}

		/**
		 * Reads all input values from the logging table. Called automatically by AdvantageKit's logging
		 * framework.
		 *
		 * @param table The LogTable to read values from
		 */
		@Override
		public void fromLog(LogTable table) {
			armAngleDegrees = table.get("ArmAngleDegrees", armAngleDegrees);
			armMotorCurrent = table.get("ArmMotorCurrent", armMotorCurrent);
			wheelMotorCurrent = table.get("wheelMotorCurrent", wheelMotorCurrent);
			wheelRPM = table.get("wheelRPM", wheelRPM);
			sensor = table.get("Sensor", sensor);
		}
	}

	/**
	 * Updates the input object with the latest sensor readings and motor feedback. This method should
	 * be called periodically to refresh all telemetry data.
	 *
	 * @param inputs The IntakeInputs object to update with current values
	 */
	public void updateInputs(IntakeInputs inputs);

	/**
	 * Sets the target angle for the intake arm using closed-loop position control.
	 *
	 * @param angle The target angle in degrees
	 */
	public void setArmAngle(double angle);

	/**
	 * Sets the speed of the intake wheels.
	 *
	 * @param speed The speed setpoint (-1.0 to 1.0, where positive is intake direction)
	 */
	public void setIntakeSpeed(double speed);

	/**
	 * Enables or disables the lower current limit for the wheel motor. Used to reduce power
	 * consumption when holding game pieces.
	 *
	 * @param enabled true to enable lower current limit, false for normal operation
	 */
	public void setLowerCurrentLimit(boolean enabled);
}
