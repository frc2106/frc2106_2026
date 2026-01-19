// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.generic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.SUB_Superstructure;

public class CMD_Superstructure extends Command {

	private final SUB_Superstructure superstructure;
	private final SUB_Superstructure.RobotState newRobotState;

	public CMD_Superstructure(
			SUB_Superstructure superstructure, 
			SUB_Superstructure.RobotState  newRobotState
			) {

		this.superstructure = superstructure;
		this.newRobotState = newRobotState;

		addRequirements(superstructure);
	}

	@Override
	public void initialize() {
		superstructure.setRobotState(newRobotState);
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {}
}
