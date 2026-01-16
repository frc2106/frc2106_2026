// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

/**
 * Robot-wide constants for subsystems and configurations.
 *
 * <p>Organized by subsystem with consistent formatting
 */
public final class RobotConstants {

	// ====================================================================
	// Robot Mode Configuration
	// ====================================================================

	/** Operating mode for the robot (REAL, SIM, or REPLAY). */
	public enum RobotMode {
		REAL, // Physical robot hardware
		SIM, // Simulation mode for testing
		REPLAY // Replay mode for log analysis
	}

	/* CANbus Configs */

	public static final CANBus CANBUS_CANIVORE = new CANBus("canivore", "./logs/canivore.hoot");

	/** Current robot operating mode. */
	public static final RobotMode ROBOT_MODE = RobotMode.SIM;

	// ====================================================================
	// Intake Subsystem
	// ====================================================================

	public final class Intake {
		// Motor Configuration
		public static final int ARM_MOTOR_ID = 21; // CAN ID for arm motor
		public static final int ARM_MOTOR_CURRENT_LIMIT = 35; // Stator current limit (A)
		public static final int WHEEL_MOTOR_ID = 22; // CAN ID for wheel motor
		public static final int WHEEL_MOTOR_CURRENT_LIMIT_NORMAL =
				55; // Normal operation current limit (A)
		public static final int WHEEL_MOTOR_CURRENT_LIMIT_LOWER =
				5; // Reduced current limit for holding (A)

		// Encoder Configuration
		public static final double ARM_ENCODER_LOOP_OFFSET = -8.6; // Encoder offset for zeroing (deg)
		public static final double ARM_ENCODER_PID_OFFSET = 7.0; // PID offset compensation (deg)
		public static final int ARM_ENCODER_FACTOR = 165; // Encoder scaling factor

		// PID Gains for Arm Position Control
		public static final double ARM_P = 0.013; // Proportional gain
		public static final double ARM_I = 0.0; // Integral gain
		public static final double ARM_D = 0.0; // Derivative gain

		// Sensors
		public static final int SENSOR_RIO_ID = 9; // RIO DIO port for beam break sensor
	}

	// ====================================================================
	// Elevator Subsystem
	// ====================================================================

	public final class Elevator {
		// Motor Configuration
		public static final int LEFT_MOTOR_ID = 10; // CAN ID for left motor
		public static final int RIGHT_MOTOR_ID = 9; // CAN ID for right motor
		public static final String CANIVORE_NAME = "canivore"; // CANivore bus name

		// Conversion Factors
		public static final double METERS_PER_MOTOR_ROTATION =
				0.025795; // Linear distance per motor rotation (m)

		// Motion Magic Configuration
		public static final double CRUISE_VELOCITY =
				2 / METERS_PER_MOTOR_ROTATION; // Maximum cruise velocity (m/s)
		public static final double ACCELERATION =
				1.5 / METERS_PER_MOTOR_ROTATION; // Maximum acceleration (m/s²)
		public static final double JERK = 0.5 / METERS_PER_MOTOR_ROTATION; // Maximum jerk (m/s³)

		// Feedforward Constants
		public static final double KS = 0.56134; // Static friction feedforward (V)
		public static final double KV = 0.089361; // Velocity feedforward (V/(m/s))
		public static final double KA = 0.0; // Acceleration feedforward (V/(m/s²))
		public static final double KG = 0.3; // Gravity compensation (V)

		// PID Gains for Position Control
		public static final double KP = 1.5; // Proportional gain (V/m)
		public static final double KI = 0.0; // Integral gain (V/(m*s))
		public static final double KD = 0.0; // Derivative gain (V/(m/s))

		// Position Limits
		public static final double MIN_HEIGHT = 0.0; // Minimum elevator height (m)
		public static final double MAX_HEIGHT = 1.85; // Maximum elevator height (m)
	}

	// ====================================================================
	// Climb Subsystem
	// ====================================================================

	public final class Climb {
		// Motor Configuration
		public static final int CLIMB_MOTOR_ID = 23; // CAN ID for climb motor
		public static final int CLIMB_MOTOR_CURRENT_LIMIT = 45; // Stator current limit (A)
	}

	// ====================================================================
	// Shooter Subsystem
	// ====================================================================

	public static class Shooter {
		// Motor Configuration
		public static final int TOP_MOTOR_ID = 20; // CAN ID for top flywheel motor
		public static final int BOTTOM_MOTOR_ID = 21; // CAN ID for bottom flywheel motor
		public static final double CURRENT_LIMIT = 60.0; // Stator current limit (A)

		// Velocity Control PID Gains
		public static final double VELOCITY_KP = 0.1; // Proportional gain
		public static final double VELOCITY_KI = 0.0; // Integral gain
		public static final double VELOCITY_KD = 0.0; // Derivative gain
		public static final double VELOCITY_KV = 0.12; // Velocity feedforward (V/(rot/s))
		public static final double VELOCITY_KS = 0.25; // Static friction feedforward (V)

		// Velocity Setpoints
		public static final double IDLE_VELOCITY_RPM = 0.0; // Idle/stopped velocity (RPM)
		public static final double SPEAKER_VELOCITY_RPM = 4000.0; // Speaker shot velocity (RPM)
		public static final double AMP_VELOCITY_RPM = 2000.0; // Amp shot velocity (RPM)
	}

	// ====================================================================
	// Indexer Subsystem
	// ====================================================================

	public static class Indexer {
		// Motor Configuration
		public static final int MOTOR_ID = 30; // CAN ID for indexer motor
		public static final double CURRENT_LIMIT = 40.0; // Stator current limit (A)

		// Sensors
		public static final int BEAM_BREAK_DIO = 0; // RIO DIO port for beam break sensor
	}
}
