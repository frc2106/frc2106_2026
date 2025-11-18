// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.lib.windingmotor.util.math.ExpDecayFF;
import frc.robot.subsystems.led.SUB_Led;
import frc.robot.subsystems.superstructure.SUB_Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.junction.Logger;

/**
 * Climb subsystem binding IO to command-based behavior and logging. Exposes helpers for direct
 * speed control, position moves with a simple controller, and a composed sequence for match climbs.
 * Uses WPILib command factories to build runEnd-based loops with an interrupting until condition
 * and sequence composition.
 */
public class SUB_Climb extends SubsystemBase {

	/** Hardware abstraction for climb (real or simulation). */
	private final IO_ClimbBase io;

	/** Telemetry container populated every loop for logging and dashboards. */
	private final IO_ClimbBase.ClimbInputs inputs = new IO_ClimbBase.ClimbInputs();

	/**
	 * Constructs the climb subsystem around the given IO layer.
	 *
	 * @param io climb IO implementation (real or simulation)
	 */
	public SUB_Climb(IO_ClimbBase io) {
		this.io = io;
	}

	/**
	 * Periodic loop: refresh IO telemetry and publish logs. This keeps dashboards synchronized and
	 * enables later replay/analysis.
	 */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Climb", inputs);
	}

	/**
	 * Direct setter for the climb motor duty cycle.
	 *
	 * @param speed duty cycle command in the range [-1.0, 1.0]
	 */
	public void setMotorSpeed(double speed) {
		io.setMotorSpeed(speed);
	}

	/**
	 * Returns the current climb motor position (rotations).
	 *
	 * @return relative position in rotations
	 */
	public double getMotorPosition() {
		return inputs.motorPosition;
	}

	/**
	 * Creates a command that drives the climb motor toward a target position using a simple
	 * exponential-decay controller. The command body is implemented with runEnd to continuously
	 * compute effort and then stop the motor on end, with until interrupting when within a deadband.
	 *
	 * @param targetPosition target rotations to move to (relative frame)
	 * @param speed absolute duty cycle scaling in [0.0, 1.0]
	 * @return a command that completes when the target position is reached within the controller’s
	 *     tolerance
	 */
	public Command goToPosition(double targetPosition, double speed) {
		final ExpDecayFF controller = new ExpDecayFF(14, 5, .05);

		return this.runEnd(
						() -> {
							double currentPosition = getMotorPosition();
							setMotorSpeed(speed * controller.calculate(currentPosition, targetPosition));
						},
						() -> setMotorSpeed(0))
				.until(
						() -> {
							double currentPosition = getMotorPosition();
							return controller.atTarget(currentPosition, targetPosition);
						});
	}

	/**
	 * Creates a one-shot command that sets the climb motor speed immediately.
	 *
	 * @param speed duty cycle command in the range [-1.0, 1.0]
	 * @return command that performs a single write to the motor setpoint
	 */
	public Command setSpeed(double speed) {
		return Commands.runOnce(() -> setMotorSpeed(speed));
	}

	/**
	 * Builds a sequential climb routine: 1) Transition superstructure to CLIMB mode, 2) extend to
	 * initial position, 3) wait for human confirmation, 4) retract to latch height; additional LED or
	 * state hooks can be composed as desired.
	 *
	 * @param confirmDownButton condition supplier that confirms continuing the sequence when true
	 * @param motorSpeed absolute duty cycle scaling used for motion commands
	 * @param led LED subsystem for user feedback (optional integration point)
	 * @param superstructure superstructure subsystem to coordinate mechanism states
	 * @return a sequential command that executes the climb sequence
	 */
	public Command climbSequence(
			java.util.function.BooleanSupplier confirmDownButton,
			double motorSpeed,
			SUB_Led led,
			SUB_Superstructure superstructure) {
		return Commands.sequence(
				// Move the robot into a safe/clear climb configuration
				new CMD_Superstructure(superstructure, SuperstructureState.CLIMB),
				// Extend to the staging position for hook engagement
				goToPosition(3.42, motorSpeed),
				// Wait for operator confirmation (e.g., hooks aligned/confirmed)
				Commands.waitUntil(confirmDownButton),
				// Retract to pull the robot up to the bar
				goToPosition(-6.5, motorSpeed) // original comment suggested -7.1 as alternative
				);
	}
}
