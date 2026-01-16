// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.RobotConstants;

/**
 * Real hardware implementation of the elevator IO using Phoenix 6 TalonFX controllers. Uses
 * PositionVoltage closed-loop control with feedforward terms and Motion Magic-like motion
 * constraints while exposing voltage control for SysId and direct testing scenarios.
 */
public class IO_ElevatorReal implements IO_ElevatorBase {

	/** Left elevator TalonFX (device ID and bus are defined for the “canivore” CAN bus). */
	private final TalonFX leftMotor_10;

	/** Right elevator TalonFX (device ID and bus are defined for the “canivore” CAN bus). */
	private final TalonFX rightMotor_9;

	/** Reusable position-voltage control request for efficient closed-loop updates. */
	private final PositionVoltage posVol;

	/**
	 * Constructs and configures the elevator motors and control request. Applies mechanism scaling,
	 * slot gains, gravity compensation type, motion constraints, and soft limits, then initializes
	 * position request objects for subsequent setpoint commands.
	 */
	public IO_ElevatorReal() {
		leftMotor_10 = new TalonFX(10, RobotConstants.CANBUS_CANIVORE);
		rightMotor_9 = new TalonFX(9, RobotConstants.CANBUS_CANIVORE);
		var motorConfigs = new TalonFXConfiguration();

		// Scale sensor to mechanism (1 rotation sensor = 1 rotation mechanism output).
		// Additional external gearing or spool scaling is handled in code via
		// METERS_PER_MOTOR_ROTATION.
		motorConfigs.Feedback.SensorToMechanismRatio = 1.0;

		// Configure Slot0 gains and feedforward used by PositionVoltage control.
		var slot0Configs = motorConfigs.Slot0;
		slot0Configs.kS =
				RobotConstants.Elevator
						.KS; // Static friction compensation (V per rot/s ≈ threshold to start moving)
		slot0Configs.kV = RobotConstants.Elevator.KV; // Velocity feedforward (V per rot/s)
		slot0Configs.kA = RobotConstants.Elevator.KA; // Acceleration feedforward (V per rot/s^2)
		slot0Configs.kP = RobotConstants.Elevator.KP; // Proportional gain (error → output)
		slot0Configs.kI = RobotConstants.Elevator.KI; // Integral gain (steady-state trim)
		slot0Configs.kD = RobotConstants.Elevator.KD; // Derivative gain (damping)
		slot0Configs.kG = RobotConstants.Elevator.KG; // Gravity compensation (elevator-specific model)
		slot0Configs.GravityType = GravityTypeValue.Elevator_Static; // Use elevator gravity model

		// Configure motion “cruise/accel/jerk” envelope (Phoenix 6 Motion Magic-like behavior).
		var motionMagicConfigs = motorConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = RobotConstants.Elevator.CRUISE_VELOCITY;
		motionMagicConfigs.MotionMagicAcceleration = RobotConstants.Elevator.ACCELERATION;
		motionMagicConfigs.MotionMagicJerk = RobotConstants.Elevator.JERK;
		motionMagicConfigs.MotionMagicExpo_kA = RobotConstants.Elevator.KA;
		motionMagicConfigs.MotionMagicExpo_kV = RobotConstants.Elevator.KV;

		// Apply software soft limits in rotations:
		// Note: Forward threshold is converted from meters to rotations via the constant,
		// while reverse threshold should be in rotations as well; ensure MIN_HEIGHT matches units.
		motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
				RobotConstants.Elevator.MAX_HEIGHT / RobotConstants.Elevator.METERS_PER_MOTOR_ROTATION;
		motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
				RobotConstants.Elevator.MIN_HEIGHT; // Ensure units are rotations.

		// Reset encoders to a known zero for consistent startup reference.
		leftMotor_10.setPosition(0);
		rightMotor_9.setPosition(0);

		// Apply configuration to both motors, including inversion and neutral dead-brake.
		setupMotors(motorConfigs);

		// Create a reusable position-voltage request, bound to slot 0 gains.
		// PositionVoltage sets a position setpoint in rotations with a voltage-based control output.
		posVol = new PositionVoltage(0).withSlot(0);
	}

	/**
	 * Polls hardware for position, velocity, acceleration, setpoint mirror, voltages, and currents.
	 * Converts rotations to meters for human-readable dashboards/analysis and logs via AdvantageKit.
	 *
	 * @param inputs structure to populate with current measurements and commanded values
	 */
	@Override
	public void updateInputs(ElevatorInputs inputs) {
		// Convert raw sensor rotations to meters for height.
		inputs.heightM =
				leftMotor_10.getPosition().getValueAsDouble()
						* RobotConstants.Elevator.METERS_PER_MOTOR_ROTATION;

		// Raw rotations for debugging/validation.
		inputs.rotations = leftMotor_10.getPosition().getValueAsDouble();

		// Convert velocity and acceleration to linear units.
		inputs.velocityMPS =
				leftMotor_10.getVelocity().getValueAsDouble()
						* RobotConstants.Elevator.METERS_PER_MOTOR_ROTATION;
		inputs.accelerationMPS2 =
				leftMotor_10.getAcceleration().getValueAsDouble()
						* RobotConstants.Elevator.METERS_PER_MOTOR_ROTATION;

		// Mirror the last requested position in meters for observability.
		inputs.setpointM = posVol.Position * RobotConstants.Elevator.METERS_PER_MOTOR_ROTATION;

		// Applied voltages and supply currents for power introspection.
		inputs.leftMotorVoltage = leftMotor_10.getMotorVoltage().getValueAsDouble();
		inputs.rightMotorVoltage = rightMotor_9.getMotorVoltage().getValueAsDouble();
		inputs.leftMotorCurrent = leftMotor_10.getSupplyCurrent().getValueAsDouble();
		inputs.rightMotorCurrent = rightMotor_9.getSupplyCurrent().getValueAsDouble();
	}

	/**
	 * Commands a closed-loop position setpoint in meters by converting to rotations and updating the
	 * control request. Uses PositionVoltage so the controller computes a voltage output considering
	 * gains and feedforward terms.
	 *
	 * @param newPositionM target height in meters
	 */
	@Override
	public void setPositionM(double newPositionM) {
		// Convert meters to motor rotations using the defined kinematics factor.
		double targetRot = newPositionM / RobotConstants.Elevator.METERS_PER_MOTOR_ROTATION;

		// Update the reusable control request.
		// NOTE: Phoenix 6 control requests expose fields and builder-style “with” methods; capturing
		// the return value guarantees the updated request is sent on the next setControl call.
		// See PositionVoltage docs for field semantics (Position in rotations).
		posVol.withPosition(targetRot);

		// Apply the same position request to both motors (mirrored elevator sides).
		leftMotor_10.setControl(posVol);
		rightMotor_9.setControl(posVol);
	}

	/**
	 * Applies a direct voltage command to both elevator motors (open-loop). Intended for use with
	 * SysId or manual testing where a controller is bypassed.
	 *
	 * @param voltage symmetrical command in volts
	 */
	@Override
	public void setVoltage(double voltage) {
		leftMotor_10.setVoltage(voltage);
		rightMotor_9.setVoltage(voltage);
	}

	/**
	 * Applies base configuration to each TalonFX, sets opposite inversions, and enables brake mode.
	 * Be careful when mutating a shared TalonFXConfiguration reference, as reusing the same instance
	 * for both motors can lead to unintended overwrites if not applied in the correct order.
	 *
	 * @param motorConfigs baseline configuration to apply to both motors before per-motor tweaks
	 */
	public void setupMotors(TalonFXConfiguration motorConfigs) {
		// For right motor: positive rotations should move elevator upward (CW+).
		var rightMotorConfig =
				motorConfigs; // WARNING: This is the same object reference as motorConfigs.
		rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		var rightMotorConfigStatus = rightMotor_9.getConfigurator().apply(motorConfigs);

		if (rightMotorConfigStatus != StatusCode.OK) {
			DriverStation.reportWarning(
					"Failed to apply right motor configuration: " + rightMotorConfigStatus, false);
		}

		// For left motor: positive rotations should also move elevator upward but requires opposite
		// sign (CCW+).
		var leftMotorConfig = motorConfigs; // WARNING: Same shared object; last write wins if reused.
		leftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		var leftMotorConfigStatus = leftMotor_10.getConfigurator().apply(motorConfigs);

		if (leftMotorConfigStatus != StatusCode.OK) {
			DriverStation.reportWarning(
					"Failed to apply left motor configuration: " + leftMotorConfigStatus, false);
		}

		// Engage brake mode on both motors to hold position when neutral.
		leftMotor_10.setNeutralMode(NeutralModeValue.Brake);
		rightMotor_9.setNeutralMode(NeutralModeValue.Brake);

		// NOTE: To avoid shared-object pitfalls, consider creating two separate TalonFXConfiguration
		// instances or applying per-motor overrides using targeted config objects before apply.
	}
}
