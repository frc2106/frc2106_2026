// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.windingmotor.drive.module;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.LIB_DriveConstants;
import frc.robot.lib.windingmotor.drive.module.IO_ModuleBase.ModuleInputs;

/**
 * Physics simulation implementation of swerve module IO.
 *
 * <p>Simulates the complete swerve module using WPILib's DCMotorSim: - Drive motor: Velocity
 * control with feedforward characterization - Turn motor: Position control with wrap-around - Both
 * motors: Voltage saturation, current draw, and physics-based dynamics
 *
 * <p>The simulation models include: - Motor inductance and resistance - Gearbox reduction - Moment
 * of inertia - Voltage saturation (±12V) - Current draw calculation
 *
 * <p>Simulation parameters are derived from TunerConstants but can be overridden locally for more
 * accurate physics modeling. This allows testing autonomous routines and control logic without
 * hardware.
 */
public class IO_ModuleSim implements IO_ModuleBase {

	private static final double DRIVE_KP = 0.05;
	private static final double DRIVE_KD = 0.0;
	private static final double DRIVE_KS = 0.0;
	private static final double TURN_KP = 8.0;
	private static final double TURN_KD = 0.0;
	private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
	private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

	private final DCMotorSim driveSim;
	private final DCMotorSim turnSim;

	// Derived per-module so KV matches the actual gear ratio
	private final double driveKV;

	private boolean driveClosedLoop = false;
	private boolean turnClosedLoop = false;
	private final PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
	private final PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
	private double driveFFVolts = 0.0;
	private double driveAppliedVolts = 0.0;
	private double turnAppliedVolts = 0.0;

	private double lastUpdateTimestamp = Timer.getFPGATimestamp();

	public IO_ModuleSim(
			SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
					constants) {

		double wheelRadiusMeters = constants.WheelRadius;

		// Free speed at mechanism shaft from your real robot's known top speed
		double freeSpeedRadPerSec =
				LIB_DriveConstants.kSpeedAt12Volts.in(MetersPerSecond) / wheelRadiusMeters;

		// Back-calculate what gear ratio would produce this free speed with a Kraken X60 FOC.
		// Kraken X60 FOC free speed at rotor = KvRadPerSecPerVolt * 12V
		double rotorFreeSpeedRadPerSec = DRIVE_GEARBOX.KvRadPerSecPerVolt * 12.0;
		double effectiveGearRatio = rotorFreeSpeedRadPerSec / freeSpeedRadPerSec;

		// Use a realistic inertia — 0.025 from constants is too high and kills acceleration.
		// 0.001 kg·m² is typical for a swerve drive wheel+module at the mechanism shaft.
		double driveInertia = 0.001;

		driveSim =
				new DCMotorSim(
						LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, driveInertia, effectiveGearRatio),
						DRIVE_GEARBOX);

		turnSim =
				new DCMotorSim(
						LinearSystemId.createDCMotorSystem(
								TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
						TURN_GEARBOX);

		// KV feedforward: volts per (mechanism rad/s)
		// At free speed the motor voltage equals back-EMF: V = freeSpeed / KvMechanism
		driveKV = effectiveGearRatio / DRIVE_GEARBOX.KvRadPerSecPerVolt;

		turnController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void updateInputs(ModuleInputs inputs) {
		double currentTime = Timer.getFPGATimestamp();
		double dt = MathUtil.clamp(currentTime - lastUpdateTimestamp, 1e-6, 0.1);
		lastUpdateTimestamp = currentTime;

		if (driveClosedLoop) {
			driveAppliedVolts =
					driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
		} else {
			driveController.reset();
		}
		if (turnClosedLoop) {
			turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
		} else {
			turnController.reset();
		}

		driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
		turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));

		driveSim.update(dt);
		turnSim.update(dt);

		inputs.driveConnected = true;
		inputs.drivePositionRad = driveSim.getAngularPositionRad();
		inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
		inputs.driveAppliedVolts = driveAppliedVolts;
		inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

		inputs.turnConnected = true;
		inputs.turnEncoderConnected = true;
		inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
		inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
		inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
		inputs.turnAppliedVolts = turnAppliedVolts;
		inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

		inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
		inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
		inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
	}

	@Override
	public void setDriveOpenLoop(double output) {
		driveClosedLoop = false;
		driveAppliedVolts = output;
	}

	@Override
	public void setTurnOpenLoop(double output) {
		turnClosedLoop = false;
		turnAppliedVolts = output;
	}

	@Override
	public void setDriveVelocity(double velocityRadPerSec) {
		driveClosedLoop = true;
		// KS * direction + KV * velocity, with KV derived from actual motor+gearbox free speed
		driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + driveKV * velocityRadPerSec;
		driveController.setSetpoint(velocityRadPerSec);
	}

	@Override
	public void setTurnPosition(Rotation2d rotation) {
		turnClosedLoop = true;
		turnController.setSetpoint(rotation.getRadians());
	}
}
