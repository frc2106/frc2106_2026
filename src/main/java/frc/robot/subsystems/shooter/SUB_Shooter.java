// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem for launching game pieces.
 *
 * <p>Uses two Kraken X60 motors in a flywheel configuration with velocity control for consistent,
 * accurate shots. The subsystem provides commands for: - Running at specific velocities - Stopping
 * the shooter - Checking if the shooter is at the target velocity
 *
 * <p>Key features: - Dual motor flywheel for high power - Velocity closed-loop control for
 * consistency - Connection monitoring with operator alerts - Comprehensive telemetry logging -
 * Velocity tolerance checking for shot readiness
 */
public class SUB_Shooter extends SubsystemBase {
	private final IO_ShooterBase io;
	private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

	/** Velocity tolerance for determining if shooter is ready (RPM). */
	private static final double VELOCITY_TOLERANCE_RPM = 50.0;

	/** Current velocity setpoint in RPM. */
	private double targetVelocityRPM = 0.0;

	// Health monitoring alerts
	private final Alert topMotorDisconnectedAlert =
			new Alert("Shooter top motor disconnected", AlertType.kError);
	private final Alert bottomMotorDisconnectedAlert =
			new Alert("Shooter bottom motor disconnected", AlertType.kError);

	/**
	 * Constructs the shooter subsystem.
	 *
	 * @param io Shooter IO implementation (real or simulation)
	 */
	public SUB_Shooter(IO_ShooterBase io) {
		this.io = io;
	}

	/**
	 * Periodic update called every 20ms.
	 *
	 * <p>Updates inputs from IO layer, processes logs, and monitors connection health.
	 */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Shooter", inputs);

		// Update connection alerts
		topMotorDisconnectedAlert.set(!inputs.topMotorConnected);
		bottomMotorDisconnectedAlert.set(!inputs.bottomMotorConnected);

		// Log derived values
		Logger.recordOutput("Shooter/TargetVelocityRPM", targetVelocityRPM);
		Logger.recordOutput("Shooter/AtTarget", isAtTargetVelocity());
		Logger.recordOutput("Shooter/AverageVelocityRPM", getAverageVelocityRPM());
	}

	/**
	 * Sets the shooter velocity to the specified RPM.
	 *
	 * @param velocityRPM Desired velocity in RPM
	 */
	public void setVelocity(double velocityRPM) {
		targetVelocityRPM = velocityRPM;
		io.setVelocity(velocityRPM);
	}

	/** Stops the shooter motors. */
	public void stop() {
		targetVelocityRPM = 0.0;
		io.stop();
	}

	/**
	 * Returns the average velocity of both shooter motors.
	 *
	 * @return Average velocity in RPM
	 */
	public double getAverageVelocityRPM() {
		return (inputs.topMotorVelocityRPM + inputs.bottomMotorVelocityRPM) / 2.0;
	}

	/**
	 * Checks if the shooter is within tolerance of the target velocity.
	 *
	 * @return True if both motors are within VELOCITY_TOLERANCE_RPM of target
	 */
	public boolean isAtTargetVelocity() {
		return Math.abs(inputs.topMotorVelocityRPM - targetVelocityRPM) < VELOCITY_TOLERANCE_RPM
				&& Math.abs(inputs.bottomMotorVelocityRPM - targetVelocityRPM) < VELOCITY_TOLERANCE_RPM;
	}

	/**
	 * Creates a command to run the shooter at a specific velocity.
	 *
	 * @param velocityRPM Desired velocity in RPM
	 * @return Command that runs the shooter at the specified velocity
	 */
	public Command runVelocity(double velocityRPM) {
		return run(() -> setVelocity(velocityRPM));
	}

	/**
	 * Creates a command to run the shooter at a velocity and wait until ready.
	 *
	 * <p>This command will: 1. Set the shooter velocity 2. Wait until both motors reach the target
	 * (within tolerance) 3. Complete when ready for shooting
	 *
	 * @param velocityRPM Desired velocity in RPM
	 * @return Command that completes when shooter is ready
	 */
	public Command spinUpAndWait(double velocityRPM) {
		return run(() -> setVelocity(velocityRPM))
				.until(this::isAtTargetVelocity)
				.withName("SpinUpShooter");
	}

	/**
	 * Creates a command to stop the shooter.
	 *
	 * @return Command that stops the shooter motors
	 */
	public Command stopCommand() {
		return runOnce(this::stop).withName("StopShooter");
	}
}
