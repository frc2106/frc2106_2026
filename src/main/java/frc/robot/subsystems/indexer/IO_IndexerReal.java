// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.RobotConstants;

/**
 * Real hardware implementation of indexer IO.
 *
 * <p>Uses: - 1x Kraken X60 motor for feeding - 1x digital beam break sensor for piece detection -
 * Simple percent output control (no PID)
 */
public class IO_IndexerReal implements IO_IndexerBase {
	// Hardware
	private final TalonFX motor;
	private final DigitalInput beamBreak;

	// Control request
	private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

	// Status signals
	private final StatusSignal<Voltage> appliedVolts;
	private final StatusSignal<Current> current;
	private final StatusSignal<Temperature> temp;

	/**
	 * Constructs and configures the indexer.
	 *
	 * <p>Configuration: - Brake mode for immediate stops - Current limit for motor protection -
	 * Inversion for correct direction
	 */
	public IO_IndexerReal() {
		// Initialize hardware
		motor = new TalonFX(RobotConstants.Indexer.MOTOR_ID);
		beamBreak = new DigitalInput(RobotConstants.Indexer.BEAM_BREAK_DIO);

		// Configure motor
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		config.CurrentLimits.StatorCurrentLimit = RobotConstants.Indexer.CURRENT_LIMIT;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		motor.getConfigurator().apply(config);

		// Initialize status signals
		appliedVolts = motor.getMotorVoltage();
		current = motor.getStatorCurrent();
		temp = motor.getDeviceTemp();

		// Set update frequency
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, appliedVolts, current, temp);
	}

	@Override
	public void updateInputs(IndexerInputs inputs) {
		// Refresh motor signals
		var status = BaseStatusSignal.refreshAll(appliedVolts, current, temp);

		// Update motor inputs
		inputs.motorConnected = status.equals(StatusCode.OK);
		inputs.motorAppliedVolts = appliedVolts.getValueAsDouble();
		inputs.motorCurrentAmps = current.getValueAsDouble();
		inputs.motorTempCelsius = temp.getValueAsDouble();

		// Update sensor inputs
		// Beam break returns false when beam is broken (piece present)
		inputs.beamBreakConnected = true; // DIO doesn't report connection status
		inputs.hasGamePiece = !beamBreak.get();
	}

	@Override
	public void setPercentOutput(double percentOutput) {
		motor.setControl(dutyCycleRequest.withOutput(percentOutput));
	}

	@Override
	public void stop() {
		setPercentOutput(0.0);
	}
}
