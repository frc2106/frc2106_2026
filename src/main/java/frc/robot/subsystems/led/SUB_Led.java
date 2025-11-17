// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.SuperstructureState;

/**
 * LED subsystem providing operator feedback through visual patterns.
 *
 * <p>Uses WPILib's AddressableLED and LEDPattern APIs to create dynamic, state-aware
 * visualizations. Patterns are selected based on: - Current superstructure state (scoring level,
 * intake mode, algae handling) - Climb mode status (waiting/ready/go sequences) - Robot pose error
 * relative to auto starting position - Disabled vs enabled state
 *
 * <p>The subsystem runs a default rainbow pattern when idle and overrides it with state-specific
 * patterns during operation. Climb mode takes highest priority when active.
 */
public class SUB_Led extends SubsystemBase {
	/** The physical LED controller on the specified PWM port. */
	private final AddressableLED ledStrip;

	/** The buffer holding color data for each LED in the strip. */
	private final AddressableLEDBuffer ledBuffer;

	/** Current superstructure state used for pattern selection. */
	private SuperstructureState.State localState;

	/** Pair indicating whether climb mode is active and which pattern to display. */
	private Pair<Boolean, LEDPattern> climbMode;

	// Define LED patterns for reuse
	private final LEDPattern rainbowPattern;
	private final LEDPattern tealFlamePattern;
	private final LEDPattern intakePattern;
	private final LEDPattern defaultPattern;

	// Public climb patterns for external command composition
	public final LEDPattern PUB_climbWaiting;
	public final LEDPattern PUB_climbReady;
	public final LEDPattern PUB_climbGo;

	/**
	 * Constructs the LED subsystem with specified port, strip length, and auto name.
	 *
	 * <p>Initializes all patterns, sets up the LED strip, and configures the default command. The
	 * strip must be started before it will display any patterns.
	 *
	 * @param port PWM port for the LED controller
	 * @param length number of LEDs in the strip
	 * @param autoName name of the current autonomous routine (for potential customization)
	 */
	public SUB_Led(int port, int length, String autoName) {
		this.localState = SuperstructureState.IDLE;

		ledStrip = new AddressableLED(port);
		ledBuffer = new AddressableLEDBuffer(length);
		ledStrip.setLength(ledBuffer.getLength());

		// Initialize patterns with distinct visual identities
		rainbowPattern = LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Percent.per(Second).of(25));

		// Teal flame effect for algae handling
		tealFlamePattern =
				LEDPattern.gradient(
								LEDPattern.GradientType.kContinuous,
								new Color(0, 255, 255), // Bright teal
								new Color(0, 128, 128) // Darker teal
								)
						.breathe(Seconds.of(0.5));

		// Intake pattern effect for coral station
		intakePattern =
				LEDPattern.gradient(
								LEDPattern.GradientType.kContinuous,
								new Color(150, 255, 255),
								new Color(150, 128, 128))
						.breathe(Seconds.of(0.5));

		// Climb patterns with distinct blink rates
		PUB_climbWaiting = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.05));
		PUB_climbReady = LEDPattern.solid(Color.kGreen);
		PUB_climbGo = LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.1));

		// Default pattern when no specific state is active
		defaultPattern = rainbowPattern;
		climbMode = Pair.of(false, defaultPattern);

		// Set default command to run rainbow pattern
		setDefaultCommand(runPattern(defaultPattern).withName("Default"));
		ledStrip.start();
	}

	/**
	 * Periodic method called every 20ms.
	 *
	 * <p>When disabled, displays pose error if the robot has moved from its auto start position. When
	 * enabled, displays climb mode pattern if active, otherwise shows state-based pattern.
	 */
	@Override
	public void periodic() {
		if (DriverStation.isDisabled()) {
			/* Do nothing */
		} else {
			// Prioritize climb mode when active
			if (climbMode.getFirst()) {
				LEDPattern pattern = climbMode.getSecond();
				pattern.applyTo(ledBuffer);
			} else {
				updateBasedOnState();
			}
		}
		ledStrip.setData(ledBuffer);
	}

	/**
	 * Updates the LED pattern based on the current superstructure state.
	 *
	 * <p>Maps each state to a distinct visual pattern: - CLIMB: Green blink for upward movement -
	 * CORAL_STATION: Intake pattern - L1-L4: Gradient patterns with unique colors - ALGAE: Teal flame
	 * effect - IDLE: Rainbow pattern
	 */
	private void updateBasedOnState() {
		String stateName = localState.getName();
		LEDPattern pattern = defaultPattern;

		// Climbing states - Green pulse indicating upward movement
		if (stateName.startsWith("CLIMB")) {
			pattern = LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.25));
		}

		// Coral station - Pattern to indicate ready for intake
		if (stateName.equals("CORAL_STATION")) {
			pattern = intakePattern;
		}

		// Different patterns for each scoring height with unique color gradients
		if (stateName.equals("L1_SCORING")) {
			pattern =
					LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlueViolet, Color.kBlue)
							.breathe(Seconds.of(0.75));
		}

		if (stateName.equals("L2_SCORING")) {
			pattern =
					LEDPattern.gradient(
									LEDPattern.GradientType.kContinuous, Color.kYellowGreen, Color.kYellow)
							.breathe(Seconds.of(0.75));
		}

		if (stateName.equals("L3_SCORING")) {
			pattern =
					LEDPattern.gradient(
									LEDPattern.GradientType.kContinuous, Color.kLawnGreen, Color.kGreenYellow)
							.breathe(Seconds.of(0.75));
		}

		if (stateName.equals("L4_SCORING")) {
			pattern =
					LEDPattern.gradient(
									LEDPattern.GradientType.kContinuous, Color.kOrangeRed, Color.kDarkOrange)
							.breathe(Seconds.of(0.75));
		}

		// Algae states - Teal flame effect
		if (stateName.startsWith("ALGAE")) {
			pattern = tealFlamePattern;
		}

		// IDLE - Rainbow pattern
		if (stateName.equals("IDLE")) {
			pattern = rainbowPattern;
		}

		pattern.applyTo(ledBuffer);
	}

	/**
	 * Updates the local state reference for pattern selection.
	 *
	 * @param newState the new superstructure state to display
	 */
	public void updateLocalState(SuperstructureState.State newState) {
		localState = newState;
	}

	/**
	 * Creates a command that applies a pattern to the LED strip.
	 *
	 * @param pattern the LEDPattern to apply
	 * @return a command that continuously applies the pattern
	 */
	public Command runPattern(LEDPattern pattern) {
		return run(() -> pattern.applyTo(ledBuffer));
	}

	/**
	 * Sets the climb mode state and associated LED pattern.
	 *
	 * @param climbMode a Pair containing (active flag, climb LED pattern)
	 * @return a command that updates the climb mode state
	 */
	public Command setClimbState(Pair<Boolean, LEDPattern> climbMode) {
		return run(() -> this.climbMode = climbMode);
	}
}
