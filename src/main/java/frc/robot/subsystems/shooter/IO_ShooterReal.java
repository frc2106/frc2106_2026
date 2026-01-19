// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

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

	private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

	public IO_ShooterReal(
			TalonFXConfiguration shooterMotorOneConfiguration,
			TalonFXConfiguration shooterMotorTwoConfiguration,
			TalonFXConfiguration turretMotorConfiguration) {

		shooterMotorOne = new TalonFX(0, RobotConstants.CANBUS_CANIVORE);
		shooterMotorOne.getConfigurator().apply(shooterMotorOneConfiguration);

		shooterMotorTwo = new TalonFX(0, RobotConstants.CANBUS_CANIVORE);
		shooterMotorTwo.getConfigurator().apply(shooterMotorTwoConfiguration);

		shooterMotorsRequest = new VelocityVoltage(0.0);

		turretMotor = new TalonFX(0, RobotConstants.CANBUS_CANIVORE);
		turretMotor.getConfigurator().apply(turretMotorConfiguration);
		turretMotorRequest = new PositionVoltage(0.0);
	}

	@Override
	public void updateInputs(ShooterInputs inputs) {

		inputs.shooterMotorOneVelocity = shooterMotorOne.getVelocity().getValueAsDouble();
		inputs.shooterMotorOneTargetVelocity = shooterMotorOne.getOutputMeasure().getValueAsDouble();
		inputs.shooterMotorOneCurrent = shooterMotorOne.getStatorCurrent().getValueAsDouble();

		inputs.shooterMotorOneVelocity = shooterMotorTwo.getVelocity().getValueAsDouble();
		inputs.shooterMotorOneTargetVelocity = shooterMotorTwo.getOutputMeasure().getValueAsDouble();
		inputs.shooterMotorOneCurrent = shooterMotorTwo.getStatorCurrent().getValueAsDouble();

		inputs.turretMotorCurrentPosition = turretMotor.getPosition().getValueAsDouble();
		inputs.turretMotorCurrentTargetPosition =
				turretMotorRequest
						.Position; // TODO: Might need to multiply by motor conversion factor in future.
		inputs.turretMotorCurrent = turretMotor.getStatorCurrent().getValueAsDouble();
	}

	@Override
	public Pair<StatusCode, StatusCode> setShooterVelocities(double velocity) {

		velocityRequest.withVelocity(velocity);

		return Pair.of(
				shooterMotorOne.setControl(velocityRequest), shooterMotorTwo.setControl(velocityRequest));
	}

	@Override
	public StatusCode setTurretPosition(Rotation2d position) {
		turretMotorRequest.withPosition(position);
		return turretMotor.setControl(turretMotorRequest);
	}
}
