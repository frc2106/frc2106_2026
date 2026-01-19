// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.LIB_ZoneConstants.ZonePose;
import frc.robot.lib.windingmotor.drive.Drive;
import frc.robot.lib.windingmotor.vision.SUB_Vision;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.indexer.SUB_Indexer;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.led.SUB_Led;

public class newSUB_Superstructure extends SubsystemBase {

	enum RobotState{
		IDLE,
		EJECTING,
		SHOOTING,
		INTAKING
	}

	private SUB_Indexer indexerRef;
	private SUB_Intake intakeRef;
	private SUB_Led ledRef;
	private SUB_Shooter shooterRef;

	private Drive driveRef;
	private SUB_Vision visionRef;	

	private CommandXboxController operatorControllerRef;

	private RobotState currentRobotState = RobotState.IDLE;


	// Robot Constants
	private final double INTAKE_MAX_EXTENSION_METERS = 0.15;


	public newSUB_Superstructure(
			SUB_Indexer inexerRef,
			SUB_Intake intakeRef,
			SUB_Led ledRef,
			SUB_Shooter shooterRef,
			Drive driveRef,
			SUB_Vision visionRef,
			CommandXboxController operatorControllerRef
			) {
		this.indexerRef = inexerRef;
		this.intakeRef = intakeRef;
		this.ledRef = ledRef;
		this.shooterRef = shooterRef;
		this.driveRef = driveRef;
		this.visionRef = visionRef;
		this.operatorControllerRef = operatorControllerRef;
	}

	@Override
	public void periodic() {
		
		// +12V is full forward
		// 0V is nothing
		// -12V is full reverse
		switch(currentRobotState){
			case IDLE:
				indexerRef.setSpinnerVoltage(0.0);
				indexerRef.setKickerVoltage (0.0);
				intakeRef.setIntakeVoltage(0.0);
				intakeRef.setSliderPosition(0.0);
				break;

			case EJECTING:
				indexerRef.setSpinnerVoltage(-5.0);
				indexerRef.setKickerVoltage (-10.0);
				intakeRef.setIntakeVoltage(-10.0);
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS);
				break; 

			case SHOOTING:
				indexerRef.setSpinnerVoltage(5.0);
				indexerRef.setKickerVoltage (10.0);
				intakeRef.setIntakeVoltage(0.0);
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS/2);
				break; 
		
			case INTAKING:
				indexerRef.setSpinnerVoltage(5.0);
				indexerRef.setKickerVoltage (0.0);
				intakeRef.setIntakeVoltage(10.0);
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS);
				break; 

		}

	}

	public void setRobotState(RobotState newRobotState){
		currentRobotState = newRobotState; 
	}

}
