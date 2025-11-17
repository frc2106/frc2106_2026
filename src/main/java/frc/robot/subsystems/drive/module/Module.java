// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.drive.ModuleInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

/**
 * Swerve module wrapper that combines IO with control logic.
 *
 * <p>Each module consists of: - A drive motor (controls wheel speed) - A turn motor (controls wheel
 * angle) - An absolute encoder (for zeroing turn position)
 *
 * <p>The Module class: 1. Updates inputs from IO layer 2. Provides odometry samples to Drive
 * subsystem 3. Optimizes setpoints to minimize movement (±90° limit) 4. Warns operators about
 * disconnected components 5. Calculates linear position/velocity from angular measurements
 *
 * <p>Odometry is collected at high frequency (250Hz on CAN FD) and provided to the Drive subsystem
 * for pose estimation.
 */
public class Module {
	private final IO_ModuleBase io;
	private final ModuleInputsAutoLogged inputs = new ModuleInputsAutoLogged();
	private final int index;
	private final SwerveModuleConstants<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			constants;

	// Alerts for disconnected components
	private final Alert driveDisconnectedAlert;
	private final Alert turnDisconnectedAlert;
	private final Alert turnEncoderDisconnectedAlert;

	/** Cached odometry positions for this cycle. */
	private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

	/**
	 * Constructs a swerve module with specified IO and constants.
	 *
	 * @param io Hardware or simulation IO implementation
	 * @param index Module index (0=FL, 1=FR, 2=BL, 3=BR)
	 * @param constants Module constants from Phoenix Tuner
	 */
	public Module(
			IO_ModuleBase io,
			int index,
			SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
					constants) {
		this.io = io;
		this.index = index;
		this.constants = constants;

		// Initialize disconnection alerts
		driveDisconnectedAlert =
				new Alert(
						"Disconnected drive motor on module " + Integer.toString(index) + ".",
						AlertType.kError);
		turnDisconnectedAlert =
				new Alert(
						"Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
		turnEncoderDisconnectedAlert =
				new Alert(
						"Disconnected turn encoder on module " + Integer.toString(index) + ".",
						AlertType.kError);
	}

	/**
	 * Periodic update called from Drive subsystem.
	 *
	 * <p>Performs: 1. Update inputs from IO layer 2. Process odometry samples 3. Update disconnection
	 * alerts
	 */
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

		// Process all odometry samples from this cycle
		int sampleCount = inputs.odometryTimestamps.length; // All signals sampled together
		odometryPositions = new SwerveModulePosition[sampleCount];
		for (int i = 0; i < sampleCount; i++) {
			// Convert angular position to linear distance (meters)
			double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
			Rotation2d angle = inputs.odometryTurnPositions[i];
			odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
		}

		// Update disconnection alerts
		driveDisconnectedAlert.set(!inputs.driveConnected);
		turnDisconnectedAlert.set(!inputs.turnConnected);
		turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
	}

	/**
	 * Runs the module with specified setpoint state.
	 *
	 * <p>Performs: 1. Optimizes angle to minimize movement (may invert velocity) 2. Sets drive
	 * velocity (rad/s) 3. Sets turn position (angle)
	 *
	 * <p>Mutates the state object during optimization.
	 *
	 * @param state Desired module state (speed and angle)
	 */
	public void runSetpoint(SwerveModuleState state) {
		// Optimize to minimize turn movement (may reverse drive direction)
		state.optimize(getAngle());

		// Set drive velocity (convert m/s to rad/s)
		io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);

		// Set turn position
		io.setTurnPosition(state.angle);
	}

	/**
	 * Runs characterization test by commanding drive voltage while holding turn at zero.
	 *
	 * <p>Used by SysId routines to characterize drive motor feedforward gains. The turn motor is held
	 * at zero angle to prevent module rotation during straight-line characterization.
	 *
	 * @param output Voltage output to drive motor (-12 to 12)
	 */
	public void runCharacterization(double output) {
		io.setDriveOpenLoop(output);
		io.setTurnOpenLoop(0);
	}

	/** Disables all outputs to motors (sets voltage to zero). */
	public void stop() {
		io.setDriveOpenLoop(0.0);
		io.setTurnOpenLoop(0.0);
	}

	/** Returns the current turn angle of the module. */
	public Rotation2d getAngle() {
		return inputs.turnPosition;
	}

	/** Returns the current drive position of the module in meters. */
	public double getPositionMeters() {
		// Convert angular position (rad) to linear distance (m)
		return inputs.drivePositionRad * constants.WheelRadius;
	}

	/** Returns the current drive velocity of the module in meters per second. */
	public double getVelocityMetersPerSec() {
		// Convert angular velocity (rad/s) to linear velocity (m/s)
		return inputs.driveVelocityRadPerSec * constants.WheelRadius;
	}

	/** Returns the module position (turn angle and drive distance). */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getPositionMeters(), getAngle());
	}

	/** Returns the module state (turn angle and drive velocity). */
	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
	}

	/** Returns the module positions received this cycle (for odometry). */
	public SwerveModulePosition[] getOdometryPositions() {
		return odometryPositions;
	}

	/** Returns the timestamps of the samples received this cycle (for odometry). */
	public double[] getOdometryTimestamps() {
		return inputs.odometryTimestamps;
	}

	/** Returns the module position in radians (for wheel radius characterization). */
	public double getWheelRadiusCharacterizationPosition() {
		return inputs.drivePositionRad;
	}

	/** Returns the module velocity in rotations/sec (Phoenix native units for feedforward). */
	public double getFFCharacterizationVelocity() {
		return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
	}
}
