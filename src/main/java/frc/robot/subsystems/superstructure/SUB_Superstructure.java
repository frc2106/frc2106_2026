// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.windingmotor.drive.Drive;
import frc.robot.lib.windingmotor.vision.SUB_Vision;
import frc.robot.subsystems.indexer.SUB_Indexer;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.led.SUB_Led;
import frc.robot.subsystems.shooter.SUB_Shooter;

public class SUB_Superstructure extends SubsystemBase {

	public enum RobotState {
		IDLE,
		EJECTING,
		SHOOTING,
		INTAKING,
		INTAKE_OFF,
		INTAKE_HALF,
		INTAKE_IN,
		READY
	}

	private SUB_Indexer indexerRef;
	private SUB_Intake intakeRef;
	private SUB_Led ledRef;
	private SUB_Shooter shooterRef;

	private Drive driveRef;
	private SUB_Vision visionRef;

	private CommandXboxController operatorControllerRef;

	private RobotState currentRobotState = RobotState.IDLE;
	private Translation2d turretTargetPose = new Translation2d(4.631, 4.031);

	private Boolean activelyShooting = false;
	private Boolean activelyReady = false;

	// Robot Constants
	private final double INTAKE_MAX_EXTENSION_METERS = 0.15;

	public SUB_Superstructure(
			SUB_Indexer inexerRef,
			SUB_Intake intakeRef,
			SUB_Led ledRef,
			SUB_Shooter shooterRef,
			Drive driveRef,
			SUB_Vision visionRef,
			CommandXboxController operatorControllerRef) {
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
		switch (currentRobotState) {
			case IDLE:
				indexerRef.setSpinnerVoltage(0.0);
				indexerRef.setKickerVoltage(0.0);
				intakeRef.setIntakeVoltage(0.0);
				intakeRef.setSliderPosition(0.0);
				shooterRef.setShooterVelocities(0.0);
				break;

			case EJECTING:
				indexerRef.setSpinnerVoltage(-5.0);
				indexerRef.setKickerVoltage(-8.0);
				intakeRef.setIntakeVoltage(0.0);
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS);
				break;

			case SHOOTING:
				break;

			case INTAKING:
				intakeRef.setIntakeVoltage(8.0);
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS);
				break;

			case INTAKE_OFF:
				intakeRef.setIntakeVoltage(0.0);

			case INTAKE_HALF:
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS / 2);

			case INTAKE_IN:
				intakeRef.setSliderPosition(0.0);

			case READY:
				indexerRef.setSpinnerVoltage(0.0);
				indexerRef.setKickerVoltage(0.0);
				break;

				/*case CENTER_TURRET:
					shooterRef.setTurretPosition(Rotation2d.kZero);
					break;

				case TURRET_LEFT:
					shooterRef.setTurretPosition(Rotation2d.kCCW_90deg);
					break;

				case TURRET_RIGHT:
					shooterRef.setTurretPosition(Rotation2d.kCW_90deg);
					break; */

		}

		demo();

		// turretLoop();
	}

	public void turretLoop() {

		// Get robot pose
		Pose2d currentRobotPose = driveRef.getPose();

		// VELOCITY

		// Get distance from our target
		double distanceMeters = currentRobotPose.getTranslation().getDistance(turretTargetPose);

		// Calculate the RPM needed to reach our target
		// This is based off a TEST equation: https://www.desmos.com/calculator/5lntkukgt6
		double velocityTargetRPM =
				(-69.53748 * Math.pow(distanceMeters, 2)) + (1183.67738 * distanceMeters) + (610.35088);

		// TURRET ANGLE

		// Calculate angle from robot to target
		double deltaX = turretTargetPose.getX() - currentRobotPose.getX();
		double deltaY = turretTargetPose.getY() - currentRobotPose.getY();

		// Field-relative angle to target
		Rotation2d angleToTarget = new Rotation2d(deltaX, deltaY);

		// Convert to robot-relative (subtract robot heading)
		Rotation2d turretAngle = angleToTarget.minus(currentRobotPose.getRotation());

		// turretAngle = turretAngle.unaryMinus();

		// Set turret position
		shooterRef.setTurretPosition(turretAngle);

		// Set the target velocity
		if (currentRobotState == RobotState.SHOOTING || activelyShooting) {

			if (currentRobotState == RobotState.SHOOTING) {
				activelyShooting = true;
			} else if (currentRobotState == RobotState.IDLE) {
				activelyShooting = false;
			}

			shooterRef.setShooterVelocities(velocityTargetRPM);

			if (turretAngle.getRadians()
							> shooterRef.getTurretPosition() - RobotConstants.Shooter.TURRET_OFFSET
					&& turretAngle.getRadians()
							< shooterRef.getTurretPosition() + RobotConstants.Shooter.TURRET_OFFSET) {

				indexerRef.setKickerVoltage(8.0);
				indexerRef.setSpinnerVoltage(8.0);

			} else {

				indexerRef.setKickerVoltage(0.0);
				indexerRef.setSpinnerVoltage(0.0);
			}
		}

		if (currentRobotState == RobotState.READY || activelyReady) {

			if (currentRobotState == RobotState.READY) {
				activelyReady = true;
			} else if (currentRobotState == RobotState.IDLE) {
				activelyReady = false;
			}

			shooterRef.setShooterVelocities(velocityTargetRPM);
		}
	}

	public void demo() {

		Rotation2d currentRobotRotation = driveRef.getRotation();

		double deltaX = operatorControllerRef.getLeftX();
		double deltaY = operatorControllerRef.getLeftY();

		Rotation2d angleToTarget = new Rotation2d(deltaX, deltaY);

		Rotation2d turretAngle = angleToTarget.minus(currentRobotRotation);

		shooterRef.setTurretPosition(turretAngle);

		if (currentRobotState == RobotState.SHOOTING || activelyShooting) {
			if (currentRobotState == RobotState.SHOOTING) {
				activelyShooting = true;
			} else if (currentRobotState == RobotState.IDLE) {
				activelyShooting = false;
			}
			shooterRef.setShooterVelocities(100);
		}
	}

	public void setRobotState(RobotState newRobotState) {
		currentRobotState = newRobotState;
	}
}
