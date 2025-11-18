// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * Base interface for shooter IO operations.
 *
 * <p>The shooter uses two Kraken motors in a flywheel configuration for launching game pieces. Both
 * motors are controlled with velocity closed-loop control for consistent shot accuracy.
 *
 * <p>Design features: - Dual motor configuration for high power output - Velocity control for
 * consistent shot speed - Independent telemetry for each motor - Connection monitoring for health
 * alerts
 *
 * <p>The IO pattern enables real hardware, simulation, and replay implementations.
 */
public interface IO_ShooterBase {
	/**
	 * Container for shooter inputs that will be logged and replayed.
	 *
	 * <p>Provides telemetry for both shooter motors including velocity, current, voltage, and
	 * connection status. The @AutoLog annotation generates ShooterIOInputsAutoLogged for AdvantageKit
	 * replay.
	 */
	@AutoLog
	public static class ShooterInputs {
		// Top motor (motor closest to intake)
		public boolean topMotorConnected = false;
		public double topMotorVelocityRPM = 0.0;
		public double topMotorAppliedVolts = 0.0;
		public double topMotorCurrentAmps = 0.0;
		public double topMotorTempCelsius = 0.0;

		// Bottom motor (motor farthest from intake)
		public boolean bottomMotorConnected = false;
		public double bottomMotorVelocityRPM = 0.0;
		public double bottomMotorAppliedVolts = 0.0;
		public double bottomMotorCurrentAmps = 0.0;
		public double bottomMotorTempCelsius = 0.0;
	}

	/**
	 * Updates the inputs object with latest shooter data.
	 *
	 * @param inputs The inputs object to populate
	 */
	public default void updateInputs(ShooterInputs inputs) {}

	/**
	 * Sets the velocity setpoint for both shooter motors.
	 *
	 * @param velocityRPM Desired velocity in RPM (revolutions per minute)
	 */
	public default void setVelocity(double velocityRPM) {}

	/** Stops both shooter motors by setting velocity to zero. */
	public default void stop() {}
}
