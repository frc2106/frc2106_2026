// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/**
 * Base interface for indexer/feeder IO operations.
 *
 * <p>The indexer controls game piece flow from intake to shooter using a single motor and beam
 * break sensor for piece detection.
 *
 * <p>Simple design: - One motor for feeding pieces - One beam break sensor for detection - Percent
 * output control (no closed-loop)
 */
public interface IO_IndexerBase {
	/** Container for indexer inputs that will be logged and replayed. */
	@AutoLog
	public static class IndexerInputs {
		public boolean motorConnected = false;
		public double motorAppliedVolts = 0.0;
		public double motorCurrentAmps = 0.0;
		public double motorTempCelsius = 0.0;

		public boolean beamBreakConnected = false;
		public boolean hasGamePiece = false;
	}

	/** Updates the inputs object with latest data. */
	public default void updateInputs(IndexerInputs inputs) {}

	/**
	 * Runs the indexer motor at the specified percent output.
	 *
	 * @param percentOutput Motor speed from -1.0 to 1.0
	 */
	public default void setPercentOutput(double percentOutput) {}

	/** Stops the indexer motor. */
	public default void stop() {}
}
