// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;

public class SUB_Intake extends SubsystemBase {

	private final IO_IntakeBase io;

	private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();


	public SUB_Intake(IO_IntakeBase io) {
		this.io = io;
	}

	@Override
	public void periodic() {

		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);
	}

	public StatusCode setIntakeVoltage(double voltage) {
		return io.setIntakeVoltage(voltage);
	}

	public StatusCode setSliderPosition(double meters) {
		return io.setSliderPosition(meters);
	}
}
