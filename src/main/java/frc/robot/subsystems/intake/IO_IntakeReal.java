// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.windingmotor.devices.IRBeamBreak;

/**
 * Real hardware implementation of the intake IO interface. Controls the REV Robotics SparkMax and
 * SparkFlex motor controllers for the intake arm and wheels, along with IR beam break sensor for
 * game piece detection. Implements closed-loop position control for the arm and configurable
 * current limits.
 *
 * @version 1.0
 * @since 2024-2025 Season
 */
public class IO_IntakeReal implements IO_IntakeBase {

	/** Motor controller for the intake arm position control. */
	private SparkMax armMotor;

	/** Motor controller for the intake wheels. */
	private SparkFlex wheelMotor;

	/** Normal operation configuration for the wheel motor. */
	private SparkFlexConfig normalWheelConfig;

	/** Lower current limit configuration for the wheel motor. */
	private SparkFlexConfig lowerWheelConfig;

	/** IR beam break sensor for detecting game pieces in the intake. */
	private IRBeamBreak sensor;

	/** Debouncer for filtering sensor noise and preventing rapid state changes. */
	private Debouncer sensorDebouncer;

	/**
	 * Constructs a new IO_IntakeReal instance. Initializes motor controllers, configures PID
	 * parameters, sets up sensor debouncing, and applies default configurations for both normal and
	 * low-power operation modes.
	 */
	public IO_IntakeReal() {
		// Initialize motor controllers with device IDs from constants
		armMotor = new SparkMax(RobotConstants.Intake.ARM_MOTOR_ID, MotorType.kBrushless);
		wheelMotor = new SparkFlex(RobotConstants.Intake.WHEEL_MOTOR_ID, MotorType.kBrushless);
		sensor = new IRBeamBreak(RobotConstants.Intake.SENSOR_RIO_ID);

		// Initialize debouncer with 110ms threshold for both rising and falling edges
		// This filters out noise and prevents false triggers from vibration or brief interruptions
		sensorDebouncer = new Debouncer(0.11, DebounceType.kBoth);

		// Configure wheel motor for normal operation
		SparkFlexConfig wheelSparkMaxConfig = new SparkFlexConfig();
		wheelSparkMaxConfig.smartCurrentLimit(RobotConstants.Intake.WHEEL_MOTOR_CURRENT_LIMIT_NORMAL);
		wheelSparkMaxConfig.idleMode(IdleMode.kBrake);
		wheelMotor.configure(
				wheelSparkMaxConfig,
				SparkBase.ResetMode.kNoResetSafeParameters,
				SparkBase.PersistMode.kPersistParameters);

		// Configure arm motor with absolute encoder and closed-loop control
		SparkMaxConfig armSparkMaxConfig = new SparkMaxConfig();
		armSparkMaxConfig.absoluteEncoder.positionConversionFactor(
				RobotConstants.Intake.ARM_ENCODER_FACTOR);
		armSparkMaxConfig.absoluteEncoder.inverted(true);
		armSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
		armSparkMaxConfig.inverted(true);
		armSparkMaxConfig.idleMode(IdleMode.kBrake);

		// Configure PID parameters for arm position control
		ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
		closedLoopConfig.p(RobotConstants.Intake.ARM_P);
		closedLoopConfig.i(RobotConstants.Intake.ARM_I);
		closedLoopConfig.d(RobotConstants.Intake.ARM_D);

		armSparkMaxConfig.apply(closedLoopConfig);

		armMotor.configure(
				armSparkMaxConfig,
				SparkBase.ResetMode.kNoResetSafeParameters,
				SparkBase.PersistMode.kPersistParameters);

		// Store normal configuration for later switching
		normalWheelConfig = wheelSparkMaxConfig;

		// Create lower current limit configuration for power saving mode
		SparkFlexConfig newLowerWheelConfig = wheelSparkMaxConfig;
		newLowerWheelConfig.smartCurrentLimit(RobotConstants.Intake.WHEEL_MOTOR_CURRENT_LIMIT_LOWER);
		lowerWheelConfig = newLowerWheelConfig;

		// Zero the arm encoder position
		armMotor.getEncoder().setPosition(0);
	}

	/**
	 * Updates the input object with current sensor readings and motor feedback. Reads arm angle,
	 * motor currents, wheel RPM, and applies debouncing to sensor input. This method should be called
	 * periodically (typically every 20ms in robot periodic).
	 *
	 * @param inputs The IntakeInputs object to populate with current values
	 */
	@Override
	public void updateInputs(IntakeInputs inputs) {
		// Read arm angle with encoder offset compensation
		inputs.armAngleDegrees =
				armMotor.getAbsoluteEncoder().getPosition() + RobotConstants.Intake.ARM_ENCODER_LOOP_OFFSET;

		// Read motor current draws
		inputs.armMotorCurrent = armMotor.getOutputCurrent();
		inputs.wheelMotorCurrent = wheelMotor.getOutputCurrent();

		// Read wheel velocity in RPM
		inputs.wheelRPM = wheelMotor.getEncoder().getVelocity();

		// Apply debouncing to sensor reading to filter noise
		inputs.sensor = sensorDebouncer.calculate(sensor.getState());
	}

	/**
	 * Commands the arm to a specific angle using closed-loop position control. Applies PID offset
	 * compensation from constants.
	 *
	 * @param angle The target angle in degrees (0 = stowed, positive = deployed)
	 */
	@Override
	public void setArmAngle(double angle) {
		armMotor
				.getClosedLoopController()
				.setReference(angle + RobotConstants.Intake.ARM_ENCODER_PID_OFFSET, ControlType.kPosition);
	}

	/**
	 * Sets the speed of the intake wheels. Positive values intake game pieces, negative values eject.
	 *
	 * @param speed The speed setpoint from -1.0 (full eject) to 1.0 (full intake)
	 */
	@Override
	public void setIntakeSpeed(double speed) {
		wheelMotor.set(speed);
	}

	/**
	 * Switches between normal and lower current limit configurations. Lower current limit reduces
	 * power consumption when holding game pieces.
	 *
	 * @param enabled true to enable lower current limit, false for normal operation limits
	 */
	@Override
	public void setLowerCurrentLimit(boolean enabled) {
		if (enabled) {
			wheelMotor.configure(
					lowerWheelConfig,
					SparkBase.ResetMode.kNoResetSafeParameters,
					SparkBase.PersistMode.kPersistParameters);
		} else {
			wheelMotor.configure(
					normalWheelConfig,
					SparkBase.ResetMode.kNoResetSafeParameters,
					SparkBase.PersistMode.kPersistParameters);
		}
	}
}
