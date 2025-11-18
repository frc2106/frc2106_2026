// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.RobotConstants;

/**
 * Real hardware implementation of shooter IO using two Kraken X60 motors.
 *
 * <p>Configuration: - Top motor: Velocity control with PID gains - Bottom motor: Velocity control
 * with PID gains - Both motors: Coast mode for free-spinning - Velocity Magic: Smooth acceleration
 * to setpoint
 *
 * <p>The motors are configured in a flywheel arrangement where: - Top motor is closer to the intake
 * - Bottom motor is farther from the intake - Both motors spin in the same direction - Velocity
 * control ensures consistent shot speed
 */
public class IO_ShooterReal implements IO_ShooterBase {
	// Hardware
	private final TalonFX topMotor;
	private final TalonFX bottomMotor;

	// Control requests (reused to minimize garbage collection)
	private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

	// Status signals for telemetry (Phoenix 6 uses typed signals)
	private final StatusSignal<AngularVelocity> topVelocity;
	private final StatusSignal<Voltage> topAppliedVolts;
	private final StatusSignal<Current> topCurrent;
	private final StatusSignal<Temperature> topTemp;

	private final StatusSignal<AngularVelocity> bottomVelocity;
	private final StatusSignal<Voltage> bottomAppliedVolts;
	private final StatusSignal<Current> bottomCurrent;
	private final StatusSignal<Temperature> bottomTemp;

	/**
	 * Constructs and configures the shooter motors.
	 *
	 * <p>Configuration process: 1. Create motor objects on the CAN bus 2. Configure PID gains for
	 * velocity control 3. Set neutral mode to coast (allows free-spinning) 4. Configure current
	 * limits for motor protection 5. Set motor inversions for correct spin direction 6. Configure
	 * status signal update frequencies
	 */
	public IO_ShooterReal() {
		// Initialize motors
		topMotor = new TalonFX(RobotConstants.Shooter.TOP_MOTOR_ID);
		bottomMotor = new TalonFX(RobotConstants.Shooter.BOTTOM_MOTOR_ID);

		// Configure top motor
		TalonFXConfiguration topConfig = new TalonFXConfiguration();
		topConfig.Slot0.kP = RobotConstants.Shooter.VELOCITY_KP;
		topConfig.Slot0.kI = RobotConstants.Shooter.VELOCITY_KI;
		topConfig.Slot0.kD = RobotConstants.Shooter.VELOCITY_KD;
		topConfig.Slot0.kV = RobotConstants.Shooter.VELOCITY_KV;
		topConfig.Slot0.kS = RobotConstants.Shooter.VELOCITY_KS;
		topConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		topConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		topConfig.CurrentLimits.StatorCurrentLimit = RobotConstants.Shooter.CURRENT_LIMIT;
		topConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		topMotor.getConfigurator().apply(topConfig);

		// Configure bottom motor
		TalonFXConfiguration bottomConfig = new TalonFXConfiguration();
		bottomConfig.Slot0.kP = RobotConstants.Shooter.VELOCITY_KP;
		bottomConfig.Slot0.kI = RobotConstants.Shooter.VELOCITY_KI;
		bottomConfig.Slot0.kD = RobotConstants.Shooter.VELOCITY_KD;
		bottomConfig.Slot0.kV = RobotConstants.Shooter.VELOCITY_KV;
		bottomConfig.Slot0.kS = RobotConstants.Shooter.VELOCITY_KS;
		bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		bottomConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		bottomConfig.CurrentLimits.StatorCurrentLimit = RobotConstants.Shooter.CURRENT_LIMIT;
		bottomConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		bottomMotor.getConfigurator().apply(bottomConfig);

		// Initialize status signals with correct types
		topVelocity = topMotor.getVelocity();
		topAppliedVolts = topMotor.getMotorVoltage();
		topCurrent = topMotor.getStatorCurrent();
		topTemp = topMotor.getDeviceTemp();

		bottomVelocity = bottomMotor.getVelocity();
		bottomAppliedVolts = bottomMotor.getMotorVoltage();
		bottomCurrent = bottomMotor.getStatorCurrent();
		bottomTemp = bottomMotor.getDeviceTemp();

		// Set update frequencies (50Hz is sufficient for shooter telemetry)
		BaseStatusSignal.setUpdateFrequencyForAll(
				50.0,
				topVelocity,
				topAppliedVolts,
				topCurrent,
				topTemp,
				bottomVelocity,
				bottomAppliedVolts,
				bottomCurrent,
				bottomTemp);
	}

	/**
	 * Updates the inputs object with latest motor telemetry.
	 *
	 * <p>Refreshes all status signals and converts units: - Velocity: rotations/sec → RPM - Voltage:
	 * direct from motor controller - Current: stator current in amps - Temperature: device
	 * temperature in Celsius
	 */
	@Override
	public void updateInputs(ShooterInputs inputs) {
		// Refresh all signals
		var topStatus = BaseStatusSignal.refreshAll(topVelocity, topAppliedVolts, topCurrent, topTemp);
		var bottomStatus =
				BaseStatusSignal.refreshAll(bottomVelocity, bottomAppliedVolts, bottomCurrent, bottomTemp);

		// Update top motor inputs
		// Phoenix 6 velocity is in rotations/sec, convert to RPM
		inputs.topMotorConnected = topStatus.equals(StatusCode.OK);
		inputs.topMotorVelocityRPM = topVelocity.getValueAsDouble() * 60.0;
		inputs.topMotorAppliedVolts = topAppliedVolts.getValueAsDouble();
		inputs.topMotorCurrentAmps = topCurrent.getValueAsDouble();
		inputs.topMotorTempCelsius = topTemp.getValueAsDouble();

		// Update bottom motor inputs
		inputs.bottomMotorConnected = bottomStatus.equals(StatusCode.OK);
		inputs.bottomMotorVelocityRPM = bottomVelocity.getValueAsDouble() * 60.0;
		inputs.bottomMotorAppliedVolts = bottomAppliedVolts.getValueAsDouble();
		inputs.bottomMotorCurrentAmps = bottomCurrent.getValueAsDouble();
		inputs.bottomMotorTempCelsius = bottomTemp.getValueAsDouble();
	}

	/**
	 * Sets both shooter motors to the specified velocity using Velocity Magic control.
	 *
	 * <p>Velocity Magic provides: - Smooth acceleration to setpoint - Consistent velocity regulation
	 * - Automatic feedforward calculation
	 *
	 * @param velocityRPM Desired velocity in RPM (revolutions per minute)
	 */
	@Override
	public void setVelocity(double velocityRPM) {
		// Convert RPM to rotations per second (Phoenix native units)
		double velocityRotPerSec = velocityRPM / 60.0;

		// Apply velocity setpoint to both motors
		topMotor.setControl(velocityRequest.withVelocity(velocityRotPerSec));
		bottomMotor.setControl(velocityRequest.withVelocity(velocityRotPerSec));
	}

	/**
	 * Stops both shooter motors by setting velocity to zero.
	 *
	 * <p>Motors will coast to a stop due to neutral mode configuration.
	 */
	@Override
	public void stop() {
		setVelocity(0.0);
	}
}
