// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem for the 2025 robot. Manages the intake arm position, wheel speed, and sensor
 * feedback for game piece manipulation. Integrates with the superstructure state machine for
 * coordinated operation. All telemetry is logged using AdvantageKit for debugging and analysis.
 *
 * <p>This subsystem provides high-level control of the intake mechanism while the IO layer handles
 * hardware-specific implementation details.
 *
 * @version 1.0
 * @since 2024-2025 Season
 */
public class SUB_Intake extends SubsystemBase {
	/** Hardware interface for intake control (real or simulated). */
	private final IO_IntakeBase io;

	/** Container for all intake sensor data and telemetry. */
	private final IO_IntakeBase.IntakeInputs inputs = new IO_IntakeBase.IntakeInputs();

	/** Current state from the superstructure state machine. */
	private SuperstructureState.State localState = SuperstructureState.IDLE;

	/**
	 * Constructs a new intake subsystem with the specified IO implementation.
	 *
	 * @param io The hardware interface implementation (IO_IntakeReal or IO_IntakeSim)
	 */
	public SUB_Intake(IO_IntakeBase io) {
		this.io = io;
	}

	/**
	 * Periodic method called every 20ms by the command scheduler. Updates motor setpoints based on
	 * current state, reads sensor data, and logs all telemetry using AdvantageKit.
	 */
	@Override
	public void periodic() {
		// Update motor setpoints based on current superstructure state
		io.setArmAngle(localState.getDeg());
		io.setIntakeSpeed(localState.getSpeed());

		// Read latest sensor data and motor feedback
		io.updateInputs(inputs);

		// Log all intake data for debugging and analysis
		Logger.processInputs("Intake", inputs);
	}

	/**
	 * Updates the intake's target state from the superstructure state machine. This determines arm
	 * angle and wheel speed commands during periodic.
	 *
	 * @param newLocalState The new state to transition to
	 */
	public void updateLocalState(SuperstructureState.State newLocalState) {
		localState = newLocalState;
	}

	/**
	 * Gets the current state of the intake subsystem.
	 *
	 * @return The current SuperstructureState.State
	 */
	public SuperstructureState.State getCurrentLocalState() {
		return localState;
	}

	/**
	 * Gets the current state of the intake sensor (debounced).
	 *
	 * @return true if a game piece is detected, false otherwise
	 */
	public boolean getSensorState() {
		return inputs.sensor;
	}

	/**
	 * Enables or disables the lower current limit for the wheel motor. Used to reduce power
	 * consumption when holding game pieces.
	 *
	 * @param enabled true to enable lower current limit, false for normal operation
	 */
	public void setLowerCurrentLimit(boolean enabled) {
		io.setLowerCurrentLimit(enabled);
	}
}
