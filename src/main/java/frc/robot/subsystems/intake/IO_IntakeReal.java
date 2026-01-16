// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.constants.RobotConstants;

public class IO_IntakeReal implements IO_IntakeBase {

	// Motors
	private final TalonFX wheelMotor;
	private final TalonFX expansionMotor;

	// Motor requests
	private final VoltageOut wheelMotorVoltageOut;
	private final PositionVoltage expansionMotorPositionVoltage;

	public IO_IntakeReal() {

		// WHEEL MOTOR
		wheelMotor = new TalonFX(/*CAN ID */ 0, RobotConstants.CANBUS_CANIVORE);
		var wheelMotorConfig = new TalonFXConfiguration();

		wheelMotorConfig.CurrentLimits.StatorCurrentLimit = 20;
		wheelMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		wheelMotor.getConfigurator().apply(wheelMotorConfig);

		wheelMotorVoltageOut = new VoltageOut(0.0);

		// EXPANSION MOTOR
		expansionMotor = new TalonFX(/*CAN ID */ 1, RobotConstants.CANBUS_CANIVORE);
		var expansionMotorConfig = new TalonFXConfiguration();

		// METERS PER ROTATION, gearing.
		expansionMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
		expansionMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		// Configure Slot0 gains, for the PID
		var expansionMotorSlotConfigs = expansionMotorConfig.Slot0;

		expansionMotorSlotConfigs.kP = 1.0;
		expansionMotorSlotConfigs.kI = 0.0;
		expansionMotorSlotConfigs.kD = 0.0;

		expansionMotor.getConfigurator().apply(expansionMotorConfig);

		expansionMotorPositionVoltage = new PositionVoltage(0.0);
	}

	@Override
	public void updateInputs(IntakeInputs inputs) {

		inputs.wheelMotorRPM = wheelMotor.getVelocity().getValueAsDouble();
		inputs.wheelMotorCurrent = wheelMotor.getStatorCurrent().getValueAsDouble();

		inputs.expansionMotorPositionMeters = expansionMotor.getPosition().getValueAsDouble();
		inputs.expansionMotorCurrent = expansionMotor.getStatorCurrent().getValueAsDouble();
	}

	/**
	 * Sets the target voltage for the intake wheel motor.
	 *
	 * @param voltage The target voltage.
	 * @return The TalonFX status code
	 */
	@Override
	public StatusCode setWheelMotorVoltage(double voltage) {

		// Update the voltage request with new voltage.
		wheelMotorVoltageOut.withOutput(voltage);

		// Set the new motor voltage request, and return status.
		return wheelMotor.setControl(wheelMotorVoltageOut);
	}

	/**
	 * Sets the target positon of the intake expansion motor in meters.
	 *
	 * @param positionMeters The new position in meters.
	 * @return The TalonFX status code
	 */
	@Override
	public StatusCode setExpansionMotorPositionMeters(double positionMeters) {

		// Update the position voltage request with new position.
		expansionMotorPositionVoltage.withPosition(positionMeters);

		// Set the new motor position voltage request, and return status.
		return expansionMotor.setControl(expansionMotorPositionVoltage);
	}
}
