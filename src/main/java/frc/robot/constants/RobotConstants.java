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
	public static final CANBus CANBUS_CANIVORE = new CANBus("canivore", "./logs/canivore.hoot");

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
					InvertedValue.Clockwise_Positive; // Which way is positive?
		}

		// Slider motor
		public static final int SLIDER_MOTOR_CAN_ID = 10;

		public static final double INTAKE_MAX_EXTENSION_METERS = 11.3; // not meters

		public static final TalonFXConfiguration SLIDER_MOTOR_CONFIG = new TalonFXConfiguration();

		static {
			SLIDER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 20; // Amps
			SLIDER_MOTOR_CONFIG.Feedback.SensorToMechanismRatio =
					1.0; // Rotations to Whatever (gear ratio)
			SLIDER_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Break or Coast
			SLIDER_MOTOR_CONFIG.MotorOutput.Inverted =
					InvertedValue.Clockwise_Positive; // Which way is positive?

			SLIDER_MOTOR_CONFIG.Slot0.kP = 1.9; // Slot 0 P value
			SLIDER_MOTOR_CONFIG.Slot0.kI = 0.1; // Slot 0 I value
			SLIDER_MOTOR_CONFIG.Slot0.kD = 0.0; // Slot 0 D value

			SLIDER_MOTOR_CONFIG.Slot0.kS = 0.2;
			SLIDER_MOTOR_CONFIG.Slot0.kV = 0.3;

			SLIDER_MOTOR_CONFIG.Slot0.kG = -0.155;
		}

		public static final int SENSOR_RIO_ID = 9; // RIO DIO port for beam break sensor
	}

	// Shooter-----------------------------------------------------------------------------------------------------
	public static class Shooter {

		// Shooter motor one left
		public static final int SHOOTER_MOTOR_ONE_CAN_ID = 21;

		public static final TalonFXConfiguration SHOOTER_MOTOR_ONE_CONFIG = new TalonFXConfiguration();

		static {
			SHOOTER_MOTOR_ONE_CONFIG.CurrentLimits.StatorCurrentLimit = 55; // Amps
			SHOOTER_MOTOR_ONE_CONFIG.Feedback.SensorToMechanismRatio =
					1.0; // Rotations to Whatever (gear ratio)
			SHOOTER_MOTOR_ONE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Break or Coast
			SHOOTER_MOTOR_ONE_CONFIG.MotorOutput.Inverted =
					InvertedValue.CounterClockwise_Positive; // Which way is positive?

			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kP = 0.3; // Slot 0 P value
			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kI = 0.0; // Slot 0 I value
			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kD = 0.0; // Slot 0 D value

			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kS = 0.232;
			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kV = 0.119;
			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kA = 0.0;
		}

		// Shooter motor two right
		public static final int SHOOTER_MOTOR_TWO_CAN_ID = 20;

		public static final TalonFXConfiguration SHOOTER_MOTOR_TWO_CONFIG = new TalonFXConfiguration();

		static {
			SHOOTER_MOTOR_TWO_CONFIG.CurrentLimits.StatorCurrentLimit = 55; // Amps
			SHOOTER_MOTOR_TWO_CONFIG.Feedback.SensorToMechanismRatio =
					1.0; // Rotations to Whatever (gear ratio)
			SHOOTER_MOTOR_TWO_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Break or Coast
			SHOOTER_MOTOR_TWO_CONFIG.MotorOutput.Inverted =
					InvertedValue.Clockwise_Positive; // Which way is positive?

			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kP = 0.3; // Slot 0 P value
			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kI = 0.0; // Slot 0 I value
			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kD = 0.0; // Slot 0 D value

			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kS = 0.232;
			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kV = 0.119;
			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kA = 0.0;
		}

		// Turret motor
		// Turret motor
		public static final int TURRET_MOTOR_CAN_ID = 22;

		public static final double TURRET_RADIANS_MAX = 0.5; // 28.65 deg
		public static final double TURRET_RADIANS_MIN = -6.0; // -331.35 deg

		public static final double TURRET_OFFSET = 0.0;

		public static final double ROT_TO_RAD = 2.0 * Math.PI; // radians per turret rotation

		/** Converts radians to turret mechanism rotations using ROT_TO_RAD */
		public static double toRotations(double radians) {
			return radians / ROT_TO_RAD;
		}

		public static final TalonFXConfiguration TURRET_MOTOR_CONFIG = new TalonFXConfiguration();
		public static final double TURRET_SLOW_MOVE_VOLTAGE = -0.5;

		static {
			TURRET_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 40;
			TURRET_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = 154.0 / 11.0;

			TURRET_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			TURRET_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

			TURRET_MOTOR_CONFIG.Slot0.kP = 26.0;
			TURRET_MOTOR_CONFIG.Slot0.kI = 0.5;
			TURRET_MOTOR_CONFIG.Slot0.kD = 0.167;
			TURRET_MOTOR_CONFIG.Slot0.kS = 4.5;
			TURRET_MOTOR_CONFIG.Slot0.kV = 0.0;
			TURRET_MOTOR_CONFIG.Slot0.kA = 0.0;

			// Soft limits converted to rotations via toRotations()
			TURRET_MOTOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
			TURRET_MOTOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
					toRotations(TURRET_RADIANS_MAX); // 0.5 rad → ~0.0796 rot

			TURRET_MOTOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
			TURRET_MOTOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
					toRotations(TURRET_RADIANS_MIN); // -5.783 rad → ~-0.9204 rot
		}

		// Turret offset from robot center (in robot frame) - MEASURE THESE!
		public static final double TURRET_OFFSET_X_METERS = -0.474;
		public static final double TURRET_OFFSET_Y_METERS = 0.525;

		// Shooter RPM lookup table: {distance_meters, rpm}
		// TODO: Characterize these values from real shooting data!
		public static final double[][] SHOOTER_RPM_DATA = {
			{1.5, 800.0}, // Close shot
			{2.0, 1000.0},
			{2.5, 1200.0},
			{3.0, 1400.0},
			{3.5, 1600.0},
			{4.0, 1800.0},
			{4.5, 2000.0},
			{5.0, 2200.0},
			{5.5, 2400.0},
			{6.0, 2600.0} // Far shot
		};

		public static final double SHOOTER_ANGLE_RADIANS = Math.toRadians(19.044); // MEASURE YOUR ANGLE
		public static final double SHOOTER_WHEEL_DIAMETER_METERS =
				0.0762; // 3 inches between 4in and 2in
		public static final double SHOOTER_EFFICIENCY_FACTOR =
				0.8; // CHARACTERIZE THIS (0.4-0.6 typical)
	}

	// Indexer----------------------------------------------------------------------------------------------------
	public static class Indexer {

		// Spinner motor
		public static final int SPINNER_MOTOR_CAN_ID = 15;

		public static final TalonFXConfiguration SPINNER_MOTOR_CONFIG = new TalonFXConfiguration();

		static {
			SPINNER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 60; // Amps
			SPINNER_MOTOR_CONFIG.Feedback.SensorToMechanismRatio =
					1.0; // Rotations to Whatever (gear ratio)
			SPINNER_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Break or Coast
			SPINNER_MOTOR_CONFIG.MotorOutput.Inverted =
					InvertedValue.CounterClockwise_Positive; // Which way is positive?

			SPINNER_MOTOR_CONFIG.Slot0.kP = 1.0; // Slot 0 P value
			SPINNER_MOTOR_CONFIG.Slot0.kI = 0.0; // Slot 0 I value
			SPINNER_MOTOR_CONFIG.Slot0.kD = 0.0; // Slot 0 D value

			SPINNER_MOTOR_CONFIG.Slot0.kS = 0.0;
			SPINNER_MOTOR_CONFIG.Slot0.kV = 0.0;
			SPINNER_MOTOR_CONFIG.Slot0.kA = 0.0;
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

			KICKER_MOTOR_CONFIG.Slot0.kP = 0.5; // Slot 0 P value
			KICKER_MOTOR_CONFIG.Slot0.kI = 0.0; // Slot 0 I value
			KICKER_MOTOR_CONFIG.Slot0.kD = 0.0; // Slot 0 D value

			KICKER_MOTOR_CONFIG.Slot0.kS = 0.0;
			KICKER_MOTOR_CONFIG.Slot0.kV = 0.0;
			KICKER_MOTOR_CONFIG.Slot0.kA = 0.0;
		}

		public static final int BEAM_BREAK_DIO = 0; // RIO DIO port for beam break sensor
	}

	// climb
	// ---------------------------------------------------------------------------------------------------------
	public static class climber {
		public static final int CLIMBER_MOTOR_CAN_ID = 31;

		public static final double INTAKE_MAX_EXTENSION_METERS = 1.0;

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
	}
}
