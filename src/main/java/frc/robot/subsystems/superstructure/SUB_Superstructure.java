// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveCommands.ZonePose;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.led.SUB_Led;
import frc.robot.subsystems.superstructure.SuperstructureState.State;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure subsystem that coordinates all mechanism states across intake, elevator, LEDs, and
 * drive.
 *
 * <p>Responsibilities: - Holds the authoritative {@link SuperstructureState.State} and forwards
 * targets to child subsystems. - Computes context-aware dynamic targets (algae handling, eject
 * variations) based on field sensing. - Exposes command helpers that schedule state transitions
 * with WPILib’s CommandScheduler.
 *
 * <p>Design notes: - Subsystems read their targets each periodic and apply motor setpoints
 * internally; the Superstructure does not write hardware directly. - AdvantageKit logging records
 * the chosen state and key derived values for replay and post-match analysis.
 */
public class SUB_Superstructure extends SubsystemBase {

	/** Global auto-align waypoints selected from the AprilTag context; used by drive alignment. */
	public static ZonePose globalFirstPose = ZonePose.NONE;

	public static ZonePose globalSecondPose = ZonePose.NONE;

	/**
	 * The currently commanded superstructure state; this is the single source of truth for mechanism
	 * targets.
	 */
	private SuperstructureState.State currentSuperstructureState = SuperstructureState.IDLE;

	/**
	 * Dynamic eject state derived from the current geometry but with a variable wheel speed "eject"
	 * component.
	 */
	public State currentDynamicEjectState =
			SuperstructureState.createState("EJECT_DYNAMIC", 0.5, 135, .5);

	/** Dynamic algae handling state chosen from nearest tag and distance thresholds at runtime. */
	public State currentDynamicAlage = SuperstructureState.IDLE;

	/** Locally stored auto-align zone pair that percolates out to globals each loop. */
	private Pair<ZonePose, ZonePose> localAutoAlignZone = Pair.of(ZonePose.NONE, ZonePose.NONE);

	/** Child subsystems that consume superstructure state. */
	public SUB_Intake intake;

	public SUB_Elevator elevator;
	public SUB_Led led;
	public Drive drive;

	/**
	 * Last latched intake sensor state to detect transitions for behavior like “stop spinning once we
	 * have a piece.”
	 */
	private boolean previousIntakeSensorState = false;

	/** Operator controller reference for haptic endgame feedback. */
	private CommandXboxController operatorController;

	/**
	 * Construct a Superstructure that will coordinate the specified subsystems and operator
	 * interface.
	 *
	 * @param drive drive subsystem used to provide nearest-tag and distance context for dynamic
	 *     targeting
	 * @param intake intake subsystem that consumes state: wheel speed and arm angle
	 * @param elevator elevator subsystem that consumes state: target height
	 * @param led LED subsystem for operator feedback states
	 * @param operatorController operator input device used for endgame rumble feedback
	 */
	public SUB_Superstructure(
			Drive drive,
			SUB_Intake intake,
			SUB_Elevator elevator,
			SUB_Led led,
			CommandXboxController operatorController) {
		this.drive = drive;
		this.intake = intake;
		this.elevator = elevator;
		this.led = led;
		this.operatorController = operatorController;
	}

	/**
	 * Update the global superstructure state and forward to all child subsystems for consumption.
	 * Also logs values for state observability and replay with AdvantageKit.
	 *
	 * <p>WPILib commands that need a state change typically schedule a command that calls this method
	 * (e.g., via InstantCommand/CMD_Superstructure), ensuring consistency through the
	 * CommandScheduler.
	 *
	 * @param newSuperstructureState next desired state across the robot mechanisms
	 */
	public void updateSuperstructureState(SuperstructureState.State newSuperstructureState) {
		currentSuperstructureState = newSuperstructureState;

		// Forward targets to child subsystems; they apply setpoints in their periodic() calls
		elevator.updateLocalState(currentSuperstructureState);
		intake.updateLocalState(currentSuperstructureState);
		led.updateLocalState(currentSuperstructureState);

		// Log for observability and replay
		Logger.recordOutput("Superstructure/State", currentSuperstructureState.toString());
		Logger.recordOutput("Superstructure/Name", currentSuperstructureState.getName());
		Logger.recordOutput("Superstructure/HeightM", currentSuperstructureState.getHeightM());
		Logger.recordOutput("Superstructure/Deg", currentSuperstructureState.getDeg());
		Logger.recordOutput("Superstructure/Speed", currentSuperstructureState.getSpeed());
	}

	/**
	 * Create and retain a dynamic eject variant of the current state with an updated intake wheel
	 * speed, preserving current geometry so driver buttons can “modulate” eject strength without
	 * redefining poses.
	 *
	 * @param newWheelSpeed desired eject speed; height and angle carry over from current state
	 * @return the newly created dynamic eject State
	 */
	public State setAndGetEjectState(double newWheelSpeed) {
		currentDynamicEjectState =
				SuperstructureState.createState(
						"EJECT_DYNAMIC",
						currentSuperstructureState.getHeightM(),
						currentSuperstructureState.getDeg(),
						newWheelSpeed);
		return currentDynamicEjectState;
	}

	/** Accessor useful for dashboards, logs, and conditional command logic. */
	public SuperstructureState.State getCurrentSuperstructureState() {
		return currentSuperstructureState;
	}

	/** Minimum distance (meters) threshold for dynamic target selection in teleop. */
	double MIN_DIST_TELEOP = 1.5;

	/** Minimum distance (meters) threshold for dynamic target selection in auto (unused here). */
	double MIN_DIST_AUTO = 2;

	/**
	 * Main coordination loop: - Detect intake-beam transitions to switch to calmer state when a piece
	 * is secured. - Provide endgame rumble feedback to the operator controller. - Choose dynamic
	 * algae state and auto-align zone based on nearest tag and distance. - Export global zone poses
	 * and detailed logs for navigation and debugging.
	 */
	@Override
	public void periodic() {

		// If we’re at the station and the intake just detected a piece, quiet the intake until the next
		// action
		if (intake.getSensorState() != previousIntakeSensorState
				&& (currentSuperstructureState == SuperstructureState.CORAL_STATION)) {
			if (intake.getSensorState()) {
				updateSuperstructureState(SuperstructureState.IDLE_CALM);
			}
		}

		// Endgame rumble feedback: gently alert the operator when match time <= 15s
		double matchTimeRemaining = DriverStation.getMatchTime();
		boolean isEndgame = matchTimeRemaining <= 15.0 && matchTimeRemaining > 0;
		if (isEndgame) {
			// CommandXboxController inherits rumble control from GenericHID, providing left/right rumble
			// channels
			operatorController.setRumble(RumbleType.kBothRumble, 0.4);
		} else {
			operatorController.setRumble(RumbleType.kBothRumble, 0.0);
		}

		// Nearest-tag context: id + distance used for context-aware dynamic selection
		int closestTagId = drive.getRecentClosestTagData().getFirst();
		double distanceM = drive.getRecentClosestTagData().getSecond();

		double minDist = MIN_DIST_TELEOP;

		if (DriverStation.isAutonomousEnabled()) {
			// Autonomous uses its own planner; dynamic driver assists are disabled here
		} else {
			// For dashboards and operator awareness
			Logger.recordOutput("Superstructure/DynamicAlage", currentDynamicAlage.getName());

			// If no tag or too far, do nothing this cycle; otherwise classify the zone and choose a
			// dynamic algae state
			if (closestTagId == -1 || distanceM >= minDist) {
				return;
			} else {
				switch (closestTagId) {

						// Source zones: stage to pick algae on ground if empty; otherwise align to source faces
					case 2:
					case 1:
					case 12:
					case 13:
						currentDynamicAlage = SuperstructureState.ALGAE_GROUND;
						if (!intake.getSensorState()
								&& currentSuperstructureState != SuperstructureState.ALGAE_GROUND) {
							// Schedule a state transition using the WPILib scheduler so subsystem requirements
							// are respected
							CommandScheduler.getInstance()
									.schedule(new CMD_Superstructure(this, SuperstructureState.CORAL_STATION));
							localAutoAlignZone = getSource();
						}
						break;

						// Bottom face
					case 18:
					case 7:
						currentDynamicAlage = SuperstructureState.ALGAE_L3;
						localAutoAlignZone = getBottomPose();
						break;

						// Bottom right face
					case 17:
					case 8:
						currentDynamicAlage = SuperstructureState.ALGAE_L2;
						localAutoAlignZone = getBottomRight();
						break;

						// Bottom left face
					case 19:
					case 6:
						currentDynamicAlage = SuperstructureState.ALGAE_L2;
						localAutoAlignZone = getBottomLeft();
						break;

						// Top right face
					case 22:
					case 9:
						currentDynamicAlage = SuperstructureState.ALGAE_L3;
						localAutoAlignZone = getTopRightPose();
						break;

						// Top face
					case 21:
					case 10:
						currentDynamicAlage = SuperstructureState.ALGAE_L2;
						localAutoAlignZone = getTopPose();
						break;

						// Top left face
					case 20:
					case 11:
						currentDynamicAlage = SuperstructureState.ALGAE_L3;
						localAutoAlignZone = getTopLeftPose();
						break;

						// Processor: direct algae handling state, no alignment pair set here
					case 3:
					case 16:
						currentDynamicAlage = SuperstructureState.ALGAE_PROCESSOR;
						break;

					default:
						break;
				}
			}
		}

		// Example post-score transition idea was here (commented for future tuning).

		// Latch current sensor state to catch next edge
		previousIntakeSensorState = intake.getSensorState();

		// Publish global zone pair from the most recent local decision for use by the drive alignment
		// command
		globalFirstPose = localAutoAlignZone.getFirst();
		globalSecondPose = localAutoAlignZone.getSecond();

		// Log both symbolic and pose forms for easier debugging and visualization in tools
		Logger.recordOutput("AutoAlign/GlobalFirst", globalFirstPose);
		Logger.recordOutput("AutoAlign/GlobalFirstPOSE", globalFirstPose.getPose());
		Logger.recordOutput("AutoAlign/GlobalSecond", globalSecondPose);
		Logger.recordOutput("AutoAlign/GlobalSecondPOSE", globalSecondPose.getPose());
	}

	/** Current dynamic algae state, useful for dashboards and command conditions. */
	public State getCurrentDynamicAlage() {
		return currentDynamicAlage;
	}

	// Pose-pair helpers that pick alignment alternatives for two-sided faces of field structures

	private Pair<ZonePose, ZonePose> getTopPose() {
		return Pair.of(ZonePose.REEF_TOP_LEFT, ZonePose.REEF_TOP_RIGHT);
	}

	private Pair<ZonePose, ZonePose> getTopRightPose() {
		return Pair.of(ZonePose.REEF_TOP_RIGHT_BOTTOM, ZonePose.REEF_TOP_RIGHT_TOP);
	}

	private Pair<ZonePose, ZonePose> getTopLeftPose() {
		return Pair.of(ZonePose.REEF_TOP_LEFT_BOTTOM, ZonePose.REEF_TOP_LEFT_TOP);
	}

	private Pair<ZonePose, ZonePose> getBottomPose() {
		return Pair.of(ZonePose.REEF_BOTTOM_LEFT, ZonePose.REEF_BOTTOM_RIGHT);
	}

	private Pair<ZonePose, ZonePose> getBottomRight() {
		return Pair.of(ZonePose.REEF_BOTTOM_RIGHT_BOTTOM, ZonePose.REEF_BOTTOM_RIGHT_TOP);
	}

	private Pair<ZonePose, ZonePose> getBottomLeft() {
		return Pair.of(ZonePose.REEF_BOTTOM_LEFT_BOTTOM, ZonePose.REEF_BOTTOM_LEFT_TOP);
	}

	private Pair<ZonePose, ZonePose> getSource() {
		return Pair.of(ZonePose.SOURCE_LEFT, ZonePose.SOURCE_RIGHT);
	}

	/**
	 * Returns an InstantCommand that, when scheduled, immediately updates the superstructure to the
	 * current dynamic algae state; this is suitable for driver assist buttons.
	 *
	 * <p>InstantCommand runs initialize/execute/end in a single scheduler iteration, making it ideal
	 * for atomic state changes like this one.
	 */
	public Command dynamicAlage() {
		return new InstantCommand(() -> updateSuperstructureState(currentDynamicAlage), this);
	}
}