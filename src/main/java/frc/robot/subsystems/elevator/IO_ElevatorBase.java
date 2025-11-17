// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Base interface for elevator subsystem IO operations. Defines the contract for controlling and
 * monitoring the elevator including position setpoints, voltage control, and telemetry logging via
 * AdvantageKit. Implementations may target real hardware or simulation while exposing the same API.
 */
public interface IO_ElevatorBase {

	/**
	 * Telemetry container for elevator state and motor feedback. AdvantageKit will auto-generate
	 * boilerplate for toLog/fromLog when annotated with @AutoLog, enabling structured replay and
	 * analysis of inputs across runs.
	 */
	@AutoLog
	public static class ElevatorInputs implements LoggableInputs {

		/** Current elevator height in meters (converted from motor rotations). */
		public double heightM = 0.0;

		/** Raw motor rotations used as the primary position sensor. */
		public double rotations = 0.0;

		/** Linear velocity in meters per second. */
		public double velocityMPS = 0.0;

		/** Linear acceleration in meters per second squared. */
		public double accelerationMPS2 = 0.0;

		/** Last commanded position setpoint in meters (for observability). */
		public double setpointM = 0.0;

		/** Applied voltage to the left motor in volts. */
		public double leftMotorVoltage = 0.0;

		/** Applied voltage to the right motor in volts. */
		public double rightMotorVoltage = 0.0;

		/** Supply current of the left motor in amps. */
		public double leftMotorCurrent = 0.0;

		/** Supply current of the right motor in amps. */
		public double rightMotorCurrent = 0.0;

		/**
		 * Write all inputs to the AdvantageKit log table for telemetry/replay. Keys should be stable to
		 * ensure consistent analysis across sessions.
		 *
		 * @param table the logging table to populate
		 */
		@Override
		public void toLog(LogTable table) {
			table.put("heightM", heightM);
			table.put("rotations", rotations);
			table.put("velocityMPS", velocityMPS);
			table.put("accelerationMPS2", accelerationMPS2);
			table.put("setpointM", setpointM);
			table.put("leftMotorVoltage", leftMotorVoltage);
			table.put("rightMotorVoltage", rightMotorVoltage);
			table.put("leftMotorCurrent", leftMotorCurrent);
			table.put("rightMotorCurrent", rightMotorCurrent);
		}

		/**
		 * Read all inputs from the AdvantageKit log table during replay. Defaults preserve prior values
		 * when fields are missing to avoid NPEs and maintain continuity.
		 *
		 * @param table the logging table to read from
		 */
		@Override
		public void fromLog(LogTable table) {
			heightM = table.get("heightM", heightM);
			rotations = table.get("rotations", rotations);
			velocityMPS = table.get("velocityMPS", velocityMPS);
			accelerationMPS2 = table.get("accelerationMPS2", accelerationMPS2);
			setpointM = table.get("setpointM", setpointM);
			leftMotorVoltage = table.get("leftMotorVoltage", leftMotorVoltage);
			rightMotorVoltage = table.get("rightMotorVoltage", rightMotorVoltage);
			leftMotorCurrent = table.get("leftMotorCurrent", leftMotorCurrent);
			rightMotorCurrent = table.get("rightMotorCurrent", rightMotorCurrent);
		}
	}

	/**
	 * Refreshes all elevator telemetry into the provided inputs container. Should be called once per
	 * loop to keep logs and dashboards up to date.
	 *
	 * @param inputs structure to populate with the latest sensor and status values
	 */
	public void updateInputs(ElevatorInputs inputs);

	/**
	 * Applies a direct voltage command to both elevator motors. Useful for SysId characterization and
	 * open-loop testing when bypassing position loops.
	 *
	 * @param voltage command in volts applied symmetrically to the mechanism
	 */
	public void setVoltage(double voltage);

	/**
	 * Commands a closed-loop position target specified in meters. Implementations convert to
	 * rotations and send a position request to motor controllers.
	 *
	 * @param newPositionM target height in meters in the elevator coordinate frame
	 */
	public void setPositionM(double newPositionM);
}
