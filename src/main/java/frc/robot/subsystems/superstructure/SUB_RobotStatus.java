// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.windingmotor.drive.Drive;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.shooter.SUB_Shooter;

public class SUB_RobotStatus extends SubsystemBase {


	// Subsystems
	Drive drive;
	SUB_Intake intake;
	SUB_Shooter shooter;

	public SUB_Superstructure(Drive drive, SUB_Intake intake, SUB_Shooter shooter) {
	


	}
}
