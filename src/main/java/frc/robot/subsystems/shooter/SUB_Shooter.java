// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;

public class SUB_Shooter extends SubsystemBase {
	private final IO_ShooterBase io;
	private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();


	public SUB_Shooter(IO_ShooterBase io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Shooter", inputs);
	}

	public Pair<StatusCode, StatusCode> setShooterVelocities(double velocity) {
		return io.setShooterVelocities(velocity);
	}

	public StatusCode setTurretPosition(Rotation2d position) {
		return io.setTurretPosition(position);
	}
}
