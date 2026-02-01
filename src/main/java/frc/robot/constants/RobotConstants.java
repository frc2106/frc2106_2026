// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class RobotConstants {

	/** Operating mode for the robot (REAL, SIM, or REPLAY). */
	public enum RobotMode {
		REAL, // Physical robot hardware
		SIM, // Simulation mode for testing
		REPLAY // Replay mode for log analysis
	}

	// Robot global CAN BUS "rio" for robo-rio "canivore" for canivore name
	public static final CANBus CANBUS_CANIVORE = new CANBus("rio", "./logs/canivore.hoot");

	// Robot operating mode
	public static final RobotMode ROBOT_MODE = RobotMode.REAL;

	// Intake
	// ---------------------------------------------------------------------------------------------------------------
	public final class Intake {

		// Intake motor
		public static final int INTAKE_MOTOR_CAN_ID = 9;

		public static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();

		static {
			INTAKE_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 25; // Amps
			INTAKE_MOTOR_CONFIG.Feedback.SensorToMechanismRatio =
					1.0; // Rotations to Whatever (gear ratio)
			INTAKE_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Break or Coast
			INTAKE_MOTOR_CONFIG.MotorOutput.Inverted =
					InvertedValue.CounterClockwise_Positive; // Which way is positive?
		}

		// Slider motor
		public static final int SLIDER_MOTOR_CAN_ID = 10;

		public static final TalonFXConfiguration SLIDER_MOTOR_CONFIG = new TalonFXConfiguration();

		static {
			SLIDER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 20; // Amps
			SLIDER_MOTOR_CONFIG.Feedback.SensorToMechanismRatio =
					1.0; // Rotations to Whatever (gear ratio)
			SLIDER_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Break or Coast
			SLIDER_MOTOR_CONFIG.MotorOutput.Inverted =
					InvertedValue.CounterClockwise_Positive; // Which way is positive?

			SLIDER_MOTOR_CONFIG.Slot0.kP = 0.0; // Slot 0 P value
			SLIDER_MOTOR_CONFIG.Slot0.kI = 0.0; // Slot 0 I value
			SLIDER_MOTOR_CONFIG.Slot0.kD = 0.0; // Slot 0 D value
		}

		public static final int SENSOR_RIO_ID = 9; // RIO DIO port for beam break sensor
	}

	// Shooter-----------------------------------------------------------------------------------------------------
	public static class Shooter {

		// Shooter motor one
		public static final int SHOOTER_MOTOR_ONE_CAN_ID = 20;

		public static final TalonFXConfiguration SHOOTER_MOTOR_ONE_CONFIG = new TalonFXConfiguration();

		static {
			SHOOTER_MOTOR_ONE_CONFIG.CurrentLimits.StatorCurrentLimit = 20; // Amps
			SHOOTER_MOTOR_ONE_CONFIG.Feedback.SensorToMechanismRatio =
					1.0; // Rotations to Whatever (gear ratio)
			SHOOTER_MOTOR_ONE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Break or Coast
			SHOOTER_MOTOR_ONE_CONFIG.MotorOutput.Inverted =
					InvertedValue.CounterClockwise_Positive; // Which way is positive?

			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kP = 0.1; // Slot 0 P value
			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kI = 0.0; // Slot 0 I value
			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kD = 0.0; // Slot 0 D value
		}

		// Shooter motor two
		public static final int SHOOTER_MOTOR_TWO_CAN_ID = 21;

		public static final TalonFXConfiguration SHOOTER_MOTOR_TWO_CONFIG = new TalonFXConfiguration();

		static {
			SHOOTER_MOTOR_TWO_CONFIG.CurrentLimits.StatorCurrentLimit = 20; // Amps
			SHOOTER_MOTOR_TWO_CONFIG.Feedback.SensorToMechanismRatio =
					1.0; // Rotations to Whatever (gear ratio)
			SHOOTER_MOTOR_TWO_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Break or Coast
			SHOOTER_MOTOR_TWO_CONFIG.MotorOutput.Inverted =
					InvertedValue.Clockwise_Positive; // Which way is positive?

			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kP = 0.1; // Slot 0 P value
			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kI = 0.0; // Slot 0 I value
			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kD = 0.0; // Slot 0 D value
		}

		// Turret motor
		public static final int TURRET_MOTOR_CAN_ID = 22;

		public static final double TURRET_ROT_TO_TURRETRADIANS = 0.3590;
		public static final double TURRET_RADIANS_MAX = 2.26893; // 130 deg max
		public static final double TURRET_RADIANS_MIN = -2.26893; // -130 deg max

		public static final TalonFXConfiguration TURRET_MOTOR_CONFIG = new TalonFXConfiguration();

		static {
			TURRET_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 20; // Amps
			TURRET_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = 0.50819881;
			// 0.07143; // Rotations to turret angle in radians (rot) * (2pi) * (gear ratio)
			TURRET_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Break or Coast
			TURRET_MOTOR_CONFIG.MotorOutput.Inverted =
					InvertedValue.CounterClockwise_Positive; // Which way is positive?

			TURRET_MOTOR_CONFIG.Slot0.kP = 1.0; // Slot 0 P value
			TURRET_MOTOR_CONFIG.Slot0.kI = 0.0; // Slot 0 I value
			TURRET_MOTOR_CONFIG.Slot0.kD = 0.0; // Slot 0 D value

			TURRET_MOTOR_CONFIG.Slot0.kS = 0.0;
			TURRET_MOTOR_CONFIG.Slot0.kV = 0.5;
			TURRET_MOTOR_CONFIG.Slot0.kA = 0.0;

			TURRET_MOTOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
			TURRET_MOTOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TURRET_RADIANS_MAX;

			TURRET_MOTOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
			TURRET_MOTOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TURRET_RADIANS_MIN;
		}
	}

	// Indexer----------------------------------------------------------------------------------------------------
	public static class Indexer {

		// Spinner motor
		public static final int SPINNER_MOTOR_CAN_ID = 30;

		public static final TalonFXConfiguration SPINNER_MOTOR_CONFIG = new TalonFXConfiguration();

		static {
			SPINNER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 20; // Amps
			SPINNER_MOTOR_CONFIG.Feedback.SensorToMechanismRatio =
					1.0; // Rotations to Whatever (gear ratio)
			SPINNER_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Break or Coast
			SPINNER_MOTOR_CONFIG.MotorOutput.Inverted =
					InvertedValue.CounterClockwise_Positive; // Which way is positive?
		}

		// Kicker motor
		public static final int KICKER_MOTOR_CAN_ID = 29;

		public static final TalonFXConfiguration KICKER_MOTOR_CONFIG = new TalonFXConfiguration();

		static {
			KICKER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 15; // Amps
			KICKER_MOTOR_CONFIG.Feedback.SensorToMechanismRatio =
					1.0; // Rotations to Whatever (gear ratio)
			KICKER_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Break or Coast
			KICKER_MOTOR_CONFIG.MotorOutput.Inverted =
					InvertedValue.CounterClockwise_Positive; // Which way is positive?
		}

		public static final int BEAM_BREAK_DIO = 0; // RIO DIO port for beam break sensor
	}
}
