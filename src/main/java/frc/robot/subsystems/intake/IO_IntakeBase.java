// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IO_IntakeBase {

	/**
	 * Input data class for intake subsystem telemetry and state. Automatically logged using
	 * AdvantageKit's @AutoLog annotation. Contains all sensor readings and motor feedback for the
	 * intake mechanism.
	 */
	@AutoLog
	public static class IntakeInputs implements LoggableInputs {

		/** Current velocity, in RPM, of intake wheel motor. Defaults to 0.0. */
		public double wheelMotorRPM = 0.0;

		/** Expansion motor postion in meters. Defaults to 0.0. */
		public double expansionMotorPositionMeters = 0.0;

		/** Current draw of the intake wheels motor in amps. Defaults to 0.0. */
		public double wheelMotorCurrent = 0.0;

		/** Current draw of the expansion motor. Defaults to 0.0. */
		public double expansionMotorCurrent = 0.0;

		/**
		 * Writes all input values to the logging table for telemetry. Called automatically by
		 * AdvantageKit's logging framework.
		 *
		 * @param table The LogTable to write values to
		 */
		@Override
		public void toLog(LogTable table) {
			table.put("wheelMotorRPM", wheelMotorRPM);
			table.put("expansionMotorPositionMeters", expansionMotorPositionMeters);
			table.put("wheelMotorCurrent", wheelMotorCurrent);
			table.put("expansionMotorCurrent", expansionMotorCurrent);
		}

		/**
		 * Reads all input values from the logging table. Called automatically by AdvantageKit's logging
		 * framework.
		 *
		 * @param table The LogTable to read values from
		 */
		@Override
		public void fromLog(LogTable table) {
			wheelMotorRPM = table.get("wheelMotorRPM", wheelMotorRPM);
			expansionMotorPositionMeters =
					table.get("expansionMotorPositionMeters", expansionMotorPositionMeters);
			wheelMotorCurrent = table.get("wheelMotorCurrent", wheelMotorCurrent);
			expansionMotorCurrent = table.get("expansionMotorCurrent", expansionMotorCurrent);
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
	 * Sets the target voltage for the intake wheel motor.
	 *
	 * @param voltage The target voltage.
	 * @return The TalonFX status code
	 */
	public StatusCode setWheelMotorVoltage(double volage);

	/**
	 * Sets the target positon of the intake expansion motor in meters.
	 *
	 * @param positionMeters The new position in meters.
	 * @return The TalonFX status code
	 */
	public StatusCode setExpansionMotorPositionMeters(double positionMeters);
}
