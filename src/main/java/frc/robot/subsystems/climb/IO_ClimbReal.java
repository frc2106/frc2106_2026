// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.constants.RobotConstants;

/**
 * Real hardware implementation of the climb IO using a REV Spark Flex controller. Configures
 * current limit and brake mode, resets the encoder reference, and provides telemetry and duty-cycle
 * control. Uses the Spark Flex’s built-in encoder for position/velocity and output current for
 * basic health monitoring.
 */
public class IO_ClimbReal implements IO_ClimbBase {

	/** Spark Flex motor controller driving the climb mechanism. */
	private SparkFlex climbMotor;

	/**
	 * Constructs the climb IO and applies persistent configuration to the Spark Flex. Sets smart
	 * current limit to protect wiring/mechanism and enables brake to hold position when neutral.
	 */
	public IO_ClimbReal() {
		climbMotor = new SparkFlex(RobotConstants.Climb.CLIMB_MOTOR_ID, MotorType.kBrushless);

		// Configure the climb motor (current limit and brake mode are persisted to flash)
		SparkFlexConfig climbSparkFlexConfig = new SparkFlexConfig();
		climbSparkFlexConfig.smartCurrentLimit(RobotConstants.Climb.CLIMB_MOTOR_CURRENT_LIMIT);
		climbSparkFlexConfig.idleMode(IdleMode.kBrake);
		climbMotor.configure(
				climbSparkFlexConfig,
				SparkBase.ResetMode.kNoResetSafeParameters,
				SparkBase.PersistMode.kPersistParameters);

		// Reset the encoder position to establish a known zero reference at startup
		climbMotor.getEncoder().setPosition(0);
	}

	/**
	 * Populates telemetry from the Spark Flex: - Current draw for protection/diagnostics - Relative
	 * position and velocity from the integrated encoder
	 *
	 * @param inputs structure to populate with current measurements
	 */
	@Override
	public void updateInputs(ClimbInputs inputs) {
		inputs.motorCurrent = climbMotor.getOutputCurrent();
		inputs.motorPosition = climbMotor.getEncoder().getPosition();
		inputs.motorVelocity = climbMotor.getEncoder().getVelocity();
	}

	/**
	 * Sets the climb motor duty cycle in open-loop mode. Positive direction depends on mechanism
	 * orientation and any applied inversion.
	 *
	 * @param speed duty cycle command in the range [-1.0, 1.0]
	 */
	@Override
	public void setMotorSpeed(double speed) {
		climbMotor.set(speed);
	}
}
