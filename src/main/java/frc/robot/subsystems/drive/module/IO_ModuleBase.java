// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Base interface for swerve module IO operations.
 *
 * <p>Each swerve module has two motors: - Drive motor: Controls wheel speed (linear motion) - Turn
 * motor: Controls wheel angle (rotation about Z-axis)
 *
 * <p>Additionally, each module has: - Drive encoder: Integrated into drive motor (relative
 * position) - Turn encoder: Integrated into turn motor (relative position) - Absolute encoder:
 * Canandmag sensor (absolute position for zeroing)
 *
 * <p>The IO pattern enables real hardware, simulation, and replay implementations while exposing
 * identical APIs to the Module wrapper class.
 *
 * <p>Odometry is collected at high frequency (250Hz on CAN FD) via timestamped queues to achieve
 * accurate pose estimation during aggressive maneuvers.
 */
public interface IO_ModuleBase {
	/**
	 * Container for swerve module inputs that will be logged and replayed.
	 *
	 * <p>Contains all sensor data, connection status, and odometry samples. The @AutoLog annotation
	 * generates ModuleInputsAutoLogged for replay.
	 */
	@AutoLog
	public static class ModuleInputs {
		// Drive motor status
		public boolean driveConnected = false;
		public double drivePositionRad = 0.0;
		public double driveVelocityRadPerSec = 0.0;
		public double driveAppliedVolts = 0.0;
		public double driveCurrentAmps = 0.0;

		// Turn motor status
		public boolean turnConnected = false;
		public boolean turnEncoderConnected = false;
		public Rotation2d turnAbsolutePosition = new Rotation2d();
		public Rotation2d turnPosition = new Rotation2d();
		public double turnVelocityRadPerSec = 0.0;
		public double turnAppliedVolts = 0.0;
		public double turnCurrentAmps = 0.0;

		// Odometry samples (high-frequency, timestamped)
		public double[] odometryTimestamps = new double[] {};
		public double[] odometryDrivePositionsRad = new double[] {};
		public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
	}

	/** Updates the inputs object with latest hardware data. */
	public default void updateInputs(ModuleInputs inputs) {}

	/** Runs drive motor in open-loop voltage control. */
	public default void setDriveOpenLoop(double output) {}

	/** Runs turn motor in open-loop voltage control. */
	public default void setTurnOpenLoop(double output) {}

	/**
	 * Runs drive motor in closed-loop velocity control.
	 *
	 * @param velocityRadPerSec Desired velocity in radians per second
	 */
	public default void setDriveVelocity(double velocityRadPerSec) {}

	/**
	 * Runs turn motor in closed-loop position control.
	 *
	 * @param rotation Desired angle
	 */
	public default void setTurnPosition(Rotation2d rotation) {}
}
