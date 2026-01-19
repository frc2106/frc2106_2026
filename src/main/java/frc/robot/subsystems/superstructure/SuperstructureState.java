// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import java.util.HashMap;
import java.util.Map;

public class SuperstructureState {
	private static final Map<String, State> dynamicStates = new HashMap<>();

	public static class State {
		private final String name;
		private final int intake;
		private final int deg;
		private final double speed;

		private State(String name, int intakeMotorSpeed, int sliderMotorMeters, double speed) {
			this.name = name;
			this.heightM = heightM;
			this.deg = deg;
			this.speed = speed;
		}

		/** Target elevator height in meters. */
		public double getHeightM() {
			return heightM;
		}

		/** Target intake arm angle in degrees. */
		public int getDeg() {
			return deg;
		}

		/** Target intake wheel speed setpoint (units per subsystem contract). */
		public double getSpeed() {
			return speed;
		}

		/** Symbolic state name for logs and dashboards. */
		public String getName() {
			return name;
		}

		@Override
		public String toString() {
			return name;
		}
	}

	// Pre-defined states
	// Height (M), Angle (Deg), Speed (duty or RPM-equivalent per Intake contract)
	public static final State IDLE = createState("IDLE", 0.075, 11, 0.12);
	public static final State IDLE_CALM = createState("IDLE_CALM", 0.075, 11, 0.0);
	public static final State CLIMB = createState("CLIMB", 0.0, 90, 0.0);

	// Coral interaction presets
	public static final State CORAL_STATION = createState("CORAL_STATION", 0.26, 32, 0.7);
	public static final State L1_SCORING = createState("L1_SCORING", 0.05, 57, 0.0);
	public static final State L2_SCORING = createState("L2_SCORING", 0.52, 120, 0.0);
	public static final State L3_SCORING = createState("L3_SCORING", 0.92, 120, 0.0);

	public static final State L4_SCORING_AUTO = createState("L4_SCORING_AUTO", 1.642, 136, 0.0);
	public static final State L4_SCORING_TELE = createState("L4_SCORING_TELE", 1.66, 140, 0.0);

	// Clearing presets
	public static final State L2_CLEAR = createState("L2_CLEAR", 0.52, 35, 0.0);
	public static final State L3_CLEAR = createState("L3_CLEAR", 0.935, 35, 0.0);
	public static final State L4_CLEAR = createState("L4_CLEAR", 1.64, 35, 0.0);

	// Algae handling
	public static final State ALGAE_GROUND = createState("ALGAE_GROUND", 0.02, 115, .75);
	public static final State ALGAE_PROCESSOR = createState("ALGAE_PROCESSOR", 0.03, 100, 1.0);
	public static final State ALGAE_L2 = createState("ALGAE_L2", 0.51, 95, 1.0);
	public static final State ALGAE_L3 = createState("ALGAE_L3", 0.91, 95, 1.0);
	public static final State ALGAE_BARGE = createState("ALGAE_BARGE", 1.7, 45, 1.0);

	/**
	 * Factory method for creating a new state and adding it to the registry.
	 *
	 * @param name symbolic name for logging and lookup
	 * @param heightM target elevator height in meters
	 * @param deg target intake arm angle in degrees
	 * @param speed intake wheel setpoint per intake contract
	 * @return the created immutable State
	 */
	public static State createState(String name, double heightM, int deg, double speed) {
		State newState = new State(name, heightM, deg, speed);
		dynamicStates.put(name, newState);
		return newState;
	}

	/**
	 * Lookup helper for dynamic states by name.
	 *
	 * @param name state name
	 * @return state instance if present, otherwise null
	 */
	public static State getState(String name) {
		return dynamicStates.get(name);
	}

	/**
	 * Returns a defensive copy of all registered states for dashboards/tools.
	 *
	 * @return new map containing all name->state entries
	 */
	public static Map<String, State> getAllStates() {
		return new HashMap<>(dynamicStates);
	}
}
