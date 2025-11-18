// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Indexer/Feeder subsystem for controlling game piece flow.
 *
 * <p>Simple subsystem with: - One motor for feeding pieces - One beam break sensor for detection -
 * Percent output control
 *
 * <p>Typical usage: - Intake: Run forward until beam break triggers - Feed to shooter: Run forward
 * to send piece to shooter - Eject: Run backward to remove piece
 */
public class SUB_Indexer extends SubsystemBase {
	private final IO_IndexerBase io;
	private final IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

	private final Alert motorDisconnectedAlert =
			new Alert("Indexer motor disconnected", AlertType.kError);

	/**
	 * Constructs the indexer subsystem.
	 *
	 * @param io Indexer IO implementation
	 */
	public SUB_Indexer(IO_IndexerBase io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Indexer", inputs);

		// Update alerts
		motorDisconnectedAlert.set(!inputs.motorConnected);
	}

	/**
	 * Runs the indexer at the specified percent output.
	 *
	 * @param percentOutput Speed from -1.0 to 1.0
	 */
	public void setPercentOutput(double percentOutput) {
		io.setPercentOutput(percentOutput);
	}

	/** Stops the indexer motor. */
	public void stop() {
		io.stop();
	}

	/** Returns true if a game piece is detected in the indexer. */
	public boolean hasGamePiece() {
		return inputs.hasGamePiece;
	}

	// ============================================
	// Command Factories
	// ============================================

	/**
	 * Runs the indexer forward to intake a game piece. Stops when beam break is triggered.
	 *
	 * @return Command that intakes until piece is detected
	 */
	public Command intake() {
		return run(() -> setPercentOutput(0.5))
				.until(this::hasGamePiece)
				.finallyDo(() -> stop())
				.withName("IntakeIndexer");
	}

	/**
	 * Feeds a game piece to the shooter.
	 *
	 * @return Command that runs the indexer forward
	 */
	public Command feedToShooter() {
		return run(() -> setPercentOutput(0.8)).withName("FeedToShooter");
	}

	/**
	 * Ejects a game piece backward. package frc.robot.subsystems.indexer;
	 *
	 * <p>public class SUB_Indexer {
	 *
	 * <p>}
	 *
	 * @return Command that runs the indexer backward
	 */
	public Command eject() {
		return run(() -> setPercentOutput(-0.5)).withName("EjectIndexer");
	}

	/**
	 * Stops the indexer.
	 *
	 * @return Command that stops the motor
	 */
	public Command stopCommand() {
		return runOnce(this::stop).withName("StopIndexer");
	}
}
