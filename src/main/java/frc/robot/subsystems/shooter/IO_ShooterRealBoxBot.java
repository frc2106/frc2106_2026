// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;

public class IO_ShooterRealBoxBot implements IO_ShooterBase {

	private final SparkFlex shooterMotorOne;

	public IO_ShooterRealBoxBot(SparkFlexConfig shooterMotorOneConfiguration) {

		shooterMotorOne = new SparkFlex(9, MotorType.kBrushless);
		shooterMotorOne.configure(
				shooterMotorOneConfiguration,
				ResetMode.kNoResetSafeParameters,
				PersistMode.kPersistParameters);
	}

	@Override
	public void updateInputs(ShooterInputs inputs) {

		inputs.shooterMotorOneVelocity = shooterMotorOne.getEncoder().getVelocity();
		inputs.shooterMotorOneTargetVelocity = 0.0;
		inputs.shooterMotorOneCurrent = shooterMotorOne.getOutputCurrent();

		inputs.shooterMotorOneVelocity = 0.0;
		inputs.shooterMotorOneTargetVelocity = 0.0;
		inputs.shooterMotorOneCurrent = 0.0;

		inputs.turretMotorCurrentPosition = 0.0;
		inputs.turretMotorCurrentTargetPosition = 0.0;
		inputs.turretMotorCurrent = 0.0;
	}

	@Override
	public Pair<StatusCode, StatusCode> setShooterVelocities(double velocity) {

		shooterMotorOne.getClosedLoopController().setReference(velocity, ControlType.kVelocity);

		return Pair.of(StatusCode.kInvalidClass, StatusCode.kInvalidClass);
	}

	@Override
	public StatusCode setTurretPosition(Rotation2d position) {
		return StatusCode.kInvalidClass;
	}

	@Override
	public void setShooterVoltages(double voltages) {
		shooterMotorOne.set(voltages);
	}
}
