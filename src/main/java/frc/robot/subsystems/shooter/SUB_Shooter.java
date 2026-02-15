// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DigitalInput;

public class SUB_Shooter extends SubsystemBase {
	private final IO_ShooterBase io;
	private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

	 private final DigitalInput turretHomeSensor = new DigitalInput(9); // DIO 

	public SUB_Shooter(IO_ShooterBase io) {
		this.io = io;
	}

	public boolean isTurretHomeSensorTriggered() {
        // Invert if your sensor is active-low
        return turretHomeSensor.get();
    }

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Shooter", inputs);
		io.setShooterVelocities(2000);
	}

	public Pair<StatusCode, StatusCode> setShooterVelocities(double velocity) {
		return io.setShooterVelocities(velocity);
	}

	public StatusCode setTurretPosition(Rotation2d position) {
		return io.setTurretPosition(position);
	}

	public void setShooterVoltages(double voltages) {
		io.setShooterVoltages(voltages);
	}

	public double getTurretPosition() {
		return io.getTurretPosition();
	}

	/**
	 * Check if turret is within acceptable tolerance of target angle
	 *
	 * @param targetAngle The desired turret angle
	 * @return true if turret is aimed within tolerance
	 */
	public boolean isTurretAtTarget(Rotation2d targetAngle) {
		double currentAngle = getTurretPosition();
		double targetRad = targetAngle.getRadians();
		return Math.abs(targetRad - currentAngle) < RobotConstants.Shooter.TURRET_OFFSET;
	}

	/**
	 * Check if shooter wheels are at target velocity within tolerance
	 *
	 * @param targetRPM The desired shooter velocity in RPM
	 * @param toleranceRPM The acceptable velocity tolerance in RPM
	 * @return true if both shooter motors are within tolerance
	 */
	public boolean isShooterAtSpeed(double targetRPM, double toleranceRPM) {
		return Math.abs(inputs.shooterMotorOneVelocity - targetRPM) < toleranceRPM
				&& Math.abs(inputs.shooterMotorTwoVelocity - targetRPM) < toleranceRPM;
	}

	/**
	 * Check if shooter wheels are at target velocity with default tolerance (50 RPM)
	 *
	 * @param targetRPM The desired shooter velocity in RPM
	 * @return true if both shooter motors are within 50 RPM tolerance
	 */
	public boolean isShooterAtSpeed(double targetRPM) {
		return isShooterAtSpeed(targetRPM, 50.0);
	}

	/**
	 * Check if shooter is ready to fire (turret aimed and wheels at speed)
	 *
	 * @param targetAngle The desired turret angle
	 * @param targetRPM The desired shooter velocity
	 * @return true if both turret and shooter are ready
	 */
	public boolean isReadyToShoot(Rotation2d targetAngle, double targetRPM) {
		return isTurretAtTarget(targetAngle) && isShooterAtSpeed(targetRPM);
	}

	/**
	 * Get the current shooter motor velocities average
	 *
	 * @return Average velocity of both shooter motors in RPM
	 */
	public double getAverageShooterVelocity() {
		return (inputs.shooterMotorOneVelocity + inputs.shooterMotorTwoVelocity) / 2.0;
	}
}
