// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.LIB_ZoneConstants.ZonePose;
import frc.robot.lib.windingmotor.drive.Drive;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.led.SUB_Led;

public class oldSUB_Superstructure extends SubsystemBase {

	public static ZonePose globalFirstPose = ZonePose.NONE;
	public static ZonePose globalSecondPose = ZonePose.NONE;

	private SuperstructureState.State currentSuperstructureState = SuperstructureState.IDLE;

	private Pair<ZonePose, ZonePose> localAutoAlignZone = Pair.of(ZonePose.NONE, ZonePose.NONE);

	public SUB_Intake intake;

	public SUB_Elevator elevator;
	public SUB_Led led;
	public Drive drive;

	private boolean previousIntakeSensorState = false;

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
	public oldSUB_Superstructure(
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
}
