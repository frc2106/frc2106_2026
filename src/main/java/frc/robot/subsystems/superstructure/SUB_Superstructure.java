// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.windingmotor.drive.Drive;
import frc.robot.lib.windingmotor.vision.SUB_Vision;
import frc.robot.subsystems.indexer.SUB_Indexer;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.led.SUB_Led;
import frc.robot.subsystems.shooter.SUB_Shooter;
import org.littletonrobotics.junction.Logger;

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

	// Lookup tables for shooting while moving
	private final InterpolatingDoubleTreeMap shooterRPMTable;

	// Turret offset from robot center (in robot frame)
	private final Translation2d TURRET_OFFSET_ROBOT =
			new Translation2d(
					RobotConstants.Shooter.TURRET_OFFSET_X_METERS,
					RobotConstants.Shooter.TURRET_OFFSET_Y_METERS);

	public SUB_Superstructure(
			SUB_Indexer indexerRef,
			SUB_Intake intakeRef,
			SUB_Led ledRef,
			SUB_Shooter shooterRef,
			Drive driveRef,
			SUB_Vision visionRef,
			CommandXboxController operatorControllerRef) {
		this.indexerRef = indexerRef;
		this.intakeRef = intakeRef;
		this.ledRef = ledRef;
		this.shooterRef = shooterRef;
		this.driveRef = driveRef;
		this.visionRef = visionRef;
		this.operatorControllerRef = operatorControllerRef;

		// Initialize shooter RPM lookup table from constants
		shooterRPMTable = new InterpolatingDoubleTreeMap();
		for (double[] dataPoint : RobotConstants.Shooter.SHOOTER_RPM_DATA) {
			shooterRPMTable.put(dataPoint[0], dataPoint[1]);
		}
	}

	@Override
	public void periodic() {

		// Log current state
		Logger.recordOutput("Superstructure/RobotState", currentRobotState.toString());
		Logger.recordOutput("Superstructure/ActivelyShooting", activelyShooting);
		Logger.recordOutput("Superstructure/ActivelyReady", activelyReady);

		// +12V is full forward
		// 0V is nothing
		// -12V is full reverse
		switch (currentRobotState) {
			case IDLE:
				activelyShooting = false;
				activelyReady = false;
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
				activelyShooting = true;
				activelyReady = false;
				break;

			case INTAKING:
				intakeRef.setIntakeVoltage(8.0);
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS);
				break;

			case INTAKE_OFF:
				intakeRef.setIntakeVoltage(0.0);
				break;

			case INTAKE_HALF:
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS / 2);
				break;

			case INTAKE_IN:
				intakeRef.setSliderPosition(0.0);
				break;

			case READY:
				indexerRef.setSpinnerVoltage(0.0);
				indexerRef.setKickerVoltage(0.0);
				activelyReady = true;
				activelyShooting = false;
				break;
		}

		updateTurretAngle();
		updateShooterVelocity();
	}

	/**
	 * Calculate and set turret angle to compensate for robot motion. Uses virtual goal method to
	 * account for shot flight time.
	 */
	private void updateTurretAngle() {
		Pose2d robotPose = driveRef.getPose();

		// Calculate turret position in field frame
		Translation2d turretOffsetField = TURRET_OFFSET_ROBOT.rotateBy(robotPose.getRotation());
		Translation2d turretPos = robotPose.getTranslation().plus(turretOffsetField);

		// Get field-relative velocity for motion compensation
		ChassisSpeeds fieldSpeeds =
				ChassisSpeeds.fromRobotRelativeSpeeds(driveRef.getChassisSpeeds(), robotPose.getRotation());

		// Calculate virtual goal position accounting for robot motion
		Translation2d virtualGoal = calculateVirtualGoal(turretPos, fieldSpeeds);

		// Calculate angle from turret to virtual goal (field-relative)
		double deltaX = virtualGoal.getX() - turretPos.getX();
		double deltaY = virtualGoal.getY() - turretPos.getY();
		Rotation2d fieldRelativeAngle = new Rotation2d(deltaX, deltaY);

		// Convert to robot-relative angle for turret
		Rotation2d turretAngle = fieldRelativeAngle.minus(robotPose.getRotation());

		// Set turret position
		shooterRef.setTurretPosition(turretAngle);

		// Log turret aiming data
		Logger.recordOutput("Superstructure/Turret/Position", turretPos);
		Logger.recordOutput("Superstructure/Turret/TargetAngleRobotRelative", turretAngle.getDegrees());
		Logger.recordOutput(
				"Superstructure/Turret/TargetAngleFieldRelative", fieldRelativeAngle.getDegrees());
		Logger.recordOutput("Superstructure/Turret/CurrentAngle", shooterRef.getTurretPosition());
	}

	/**
	 * Calculate and set shooter wheel velocity based on distance to target. Accounts for virtual goal
	 * position when robot is moving.
	 */
	private void updateShooterVelocity() {
		Pose2d robotPose = driveRef.getPose();

		// Calculate turret position in field frame
		Translation2d turretOffsetField = TURRET_OFFSET_ROBOT.rotateBy(robotPose.getRotation());
		Translation2d turretPos = robotPose.getTranslation().plus(turretOffsetField);

		// Get field-relative velocity for motion compensation
		ChassisSpeeds fieldSpeeds =
				ChassisSpeeds.fromRobotRelativeSpeeds(driveRef.getChassisSpeeds(), robotPose.getRotation());

		// Calculate virtual goal position accounting for robot motion
		Translation2d virtualGoal = calculateVirtualGoal(turretPos, fieldSpeeds);

		// Calculate distance to virtual goal
		double distance = turretPos.getDistance(virtualGoal);

		// Look up required RPM for this distance
		double targetRPM = shooterRPMTable.get(distance);

		// Set shooter velocity
		shooterRef.setShooterVelocities(targetRPM);

		// Log shooter velocity data
		Logger.recordOutput("Superstructure/Shooter/DistanceToVirtualGoal", distance);
		Logger.recordOutput("Superstructure/Shooter/TargetRPM", targetRPM);
		Logger.recordOutput(
				"Superstructure/Shooter/AverageRPM", shooterRef.getAverageShooterVelocity());
	}

	/**
	 * Calculate virtual goal position compensating for robot motion during shot flight time. Uses
	 * physics-based flight time calculation for fixed-angle shooter.
	 *
	 * @param turretPos Current turret position in field frame
	 * @param fieldSpeeds Robot velocity in field frame (m/s)
	 * @return Virtual goal position accounting for motion compensation
	 */
	private Translation2d calculateVirtualGoal(Translation2d turretPos, ChassisSpeeds fieldSpeeds) {

		// Calculate distance to actual target
		double distanceToTarget = turretPos.getDistance(turretTargetPose);

		// Look up shooter RPM for this distance
		double targetRPM = shooterRPMTable.get(distanceToTarget);

		// Convert RPM to exit velocity (m/s)
		double wheelCircumference = Math.PI * RobotConstants.Shooter.SHOOTER_WHEEL_DIAMETER_METERS;
		double wheelSurfaceSpeed = (targetRPM / 60.0) * wheelCircumference;
		double exitVelocity = wheelSurfaceSpeed * RobotConstants.Shooter.SHOOTER_EFFICIENCY_FACTOR;

		// Calculate horizontal velocity component (fixed angle)
		double horizontalVelocity =
				exitVelocity * Math.cos(RobotConstants.Shooter.SHOOTER_ANGLE_RADIANS);

		// Calculate flight time: time = distance / horizontal velocity
		double flightTime = distanceToTarget / horizontalVelocity;

		// Calculate robot displacement during flight time
		double displacementX = flightTime * fieldSpeeds.vxMetersPerSecond;
		double displacementY = flightTime * fieldSpeeds.vyMetersPerSecond;

		// Calculate virtual goal by subtracting robot displacement during flight
		// We subtract because we need to aim "behind" our direction of motion
		double virtualX = turretTargetPose.getX() - displacementX;
		double virtualY = turretTargetPose.getY() - displacementY;

		Translation2d virtualGoal = new Translation2d(virtualX, virtualY);

		// Log motion compensation data
		Logger.recordOutput("Superstructure/MotionComp/ActualGoal", turretTargetPose);
		Logger.recordOutput("Superstructure/MotionComp/VirtualGoal", virtualGoal);
		Logger.recordOutput("Superstructure/MotionComp/DistanceToActualGoal", distanceToTarget);
		Logger.recordOutput("Superstructure/MotionComp/TargetRPM", targetRPM);
		Logger.recordOutput("Superstructure/MotionComp/ExitVelocityMPS", exitVelocity);
		Logger.recordOutput("Superstructure/MotionComp/HorizontalVelocityMPS", horizontalVelocity);
		Logger.recordOutput("Superstructure/MotionComp/FlightTimeSeconds", flightTime);
		Logger.recordOutput("Superstructure/MotionComp/RobotVelocityX", fieldSpeeds.vxMetersPerSecond);
		Logger.recordOutput("Superstructure/MotionComp/RobotVelocityY", fieldSpeeds.vyMetersPerSecond);
		Logger.recordOutput("Superstructure/MotionComp/DisplacementX", displacementX);
		Logger.recordOutput("Superstructure/MotionComp/DisplacementY", displacementY);

		// Calculate and log the compensation offset
		double compensationOffset = turretTargetPose.getDistance(virtualGoal);
		Logger.recordOutput("Superstructure/MotionComp/CompensationOffsetMeters", compensationOffset);

		return virtualGoal;
	}

	public void setRobotState(RobotState newRobotState) {
		currentRobotState = newRobotState;
	}

	/**
	 * Update the target position for the turret. Useful for autonomous or dynamic target switching.
	 */
	public void setTurretTarget(Translation2d newTarget) {
		turretTargetPose = newTarget;
		Logger.recordOutput("Superstructure/TargetPoseUpdated", newTarget);
	}

	/** Get the current turret target position. */
	public Translation2d getTurretTarget() {
		return turretTargetPose;
	}
}
