// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.RobotConstants;

public class IO_ShooterReal implements IO_ShooterBase {

	private final TalonFX shooterMotorOne;
	private final TalonFX shooterMotorTwo;
	private final VelocityVoltage shooterMotorsRequest;

	private final TalonFX turretMotor;
	private final PositionVoltage turretMotorRequest;

	public IO_ShooterReal(
			TalonFXConfiguration shooterMotorOneConfiguration,
			TalonFXConfiguration shooterMotorTwoConfiguration,
			TalonFXConfiguration turretMotorConfiguration) {

		shooterMotorOne =
				new TalonFX(
						RobotConstants.Shooter.SHOOTER_MOTOR_ONE_CAN_ID, RobotConstants.CANBUS_CANIVORE);
		shooterMotorOne.getConfigurator().apply(shooterMotorOneConfiguration);

		shooterMotorTwo =
				new TalonFX(
						RobotConstants.Shooter.SHOOTER_MOTOR_TWO_CAN_ID, RobotConstants.CANBUS_CANIVORE);
		shooterMotorTwo.getConfigurator().apply(shooterMotorTwoConfiguration);

		shooterMotorsRequest = new VelocityVoltage(0.0);

		turretMotor =
				new TalonFX(RobotConstants.Shooter.TURRET_MOTOR_CAN_ID, RobotConstants.CANBUS_CANIVORE);
		turretMotor.getConfigurator().apply(turretMotorConfiguration);
		turretMotorRequest = new PositionVoltage(0.0);

		turretMotor.setPosition(0);
	}

	@Override
	public void updateInputs(ShooterInputs inputs) {

		inputs.shooterMotorOneVelocity = shooterMotorOne.getVelocity().getValueAsDouble();
		inputs.shooterMotorOneTargetVelocity = shooterMotorsRequest.getVelocityMeasure().in(RPM);
		inputs.shooterMotorOneCurrent = shooterMotorOne.getStatorCurrent().getValueAsDouble();

		inputs.shooterMotorTwoVelocity = shooterMotorTwo.getVelocity().getValueAsDouble();
		inputs.shooterMotorTwoTargetVelocity = shooterMotorsRequest.getVelocityMeasure().in(RPM);
		inputs.shooterMotorTwoCurrent = shooterMotorTwo.getStatorCurrent().getValueAsDouble();

		inputs.turretMotorCurrentPosition = turretMotor.getPosition().getValueAsDouble();
		inputs.turretMotorCurrentTargetPosition = turretMotorRequest.Position;
		inputs.turretMotorCurrent = turretMotor.getStatorCurrent().getValueAsDouble();
	}

	@Override
	public Pair<StatusCode, StatusCode> setShooterVelocities(double velocity) {

		shooterMotorsRequest.withVelocity(velocity);

		return Pair.of(
				shooterMotorOne.setControl(shooterMotorsRequest),
				shooterMotorTwo.setControl(shooterMotorsRequest));
	}

	@Override
	public StatusCode setTurretPosition(Rotation2d position) {

		double targetRadians = position.getRadians();

		// Handle case where vision/field-relative angle might be outside our physical range
		// Normalize the angle to [-π, π] first if it's a field-relative angle
		if (Math.abs(targetRadians) > Math.PI) {
			// Normalize to [-π, π] range
			targetRadians = Math.atan2(Math.sin(targetRadians), Math.cos(targetRadians));
		}

		// Clamp to our physical limits
		if (targetRadians > RobotConstants.Shooter.TURRET_RADIANS_MAX) {
			targetRadians = RobotConstants.Shooter.TURRET_RADIANS_MAX;
		} else if (targetRadians < RobotConstants.Shooter.TURRET_RADIANS_MIN) {
			targetRadians = RobotConstants.Shooter.TURRET_RADIANS_MIN;
		}

		// Send position command - PID takes the direct path
		// With ±130° range, direct path is always valid (no wrap-around possible)
		turretMotorRequest.withPosition(targetRadians);
		return turretMotor.setControl(turretMotorRequest);
	}

	@Override
	public void setShooterVoltages(double voltages) {}
}
