// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

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
	// Simulation constants (derived from TunerConstants but can be tuned separately)
	private static final double DRIVE_KP = 0.05;
	private static final double DRIVE_KD = 0.0;
	private static final double DRIVE_KS = 0.0;
	private static final double DRIVE_KV_ROT = 0.91035; // (volt * secs) / rotation
	private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
	private static final double TURN_KP = 8.0;
	private static final double TURN_KD = 0.0;
	private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
	private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

	// Simulation models
	private final DCMotorSim driveSim;
	private final DCMotorSim turnSim;

	// Control state
	private boolean driveClosedLoop = false;
	private boolean turnClosedLoop = false;
	private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
	private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
	private double driveFFVolts = 0.0;
	private double driveAppliedVolts = 0.0;
	private double turnAppliedVolts = 0.0;

	/**
	 * Constructs the module simulation.
	 *
	 * <p>Creates DCMotorSim models for drive and turn motors using: - Motor characteristics (Kraken
	 * X60 FOC) - Gear ratios from TunerConstants - Moments of inertia from TunerConstants
	 *
	 * <p>Turn controller is configured for continuous input to handle ±180° wraparound.
	 *
	 * @param constants Swerve module constants from TunerConstants
	 */
	public IO_ModuleSim(
			SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
					constants) {
		// Create drive motor simulation model
		driveSim =
				new DCMotorSim(
						LinearSystemId.createDCMotorSystem(
								DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio),
						DRIVE_GEARBOX);

		// Create turn motor simulation model
		turnSim =
				new DCMotorSim(
						LinearSystemId.createDCMotorSystem(
								TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
						TURN_GEARBOX);

		// Enable angle wrapping for turn PID (handles ±180° boundary)
		turnController.enableContinuousInput(-Math.PI, Math.PI);
	}

	/**
	 * Updates the simulation and populates inputs.
	 *
	 * <p>Process: 1. Run closed-loop controllers if enabled 2. Apply voltage to simulation models
	 * (clamped to ±12V) 3. Update simulation state (0.02s timestep) 4. Extract sensor values and
	 * populate inputs 5. Generate odometry samples (50Hz in sim, high-freq not needed)
	 */
	@Override
	public void updateInputs(ModuleInputs inputs) {
		// Run closed-loop control if enabled
		if (driveClosedLoop) {
			// Calculate feedforward + feedback for velocity control
			driveAppliedVolts =
					driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
		} else {
			// Open loop: reset controller to prevent windup
			driveController.reset();
		}
		if (turnClosedLoop) {
			// Position control with PID
			turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
		} else {
			// Open loop: reset controller to prevent windup
			turnController.reset();
		}

		// Apply voltage to simulation (clamped to battery voltage)
		driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
		turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));

		// Update simulation physics (0.02s = 50Hz)
		driveSim.update(0.02);
		turnSim.update(0.02);

		// Update drive inputs
		inputs.driveConnected = true; // Sim is always "connected"
		inputs.drivePositionRad = driveSim.getAngularPositionRad();
		inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
		inputs.driveAppliedVolts = driveAppliedVolts;
		inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

		// Update turn inputs
		inputs.turnConnected = true;
		inputs.turnEncoderConnected = true;
		inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
		inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
		inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
		inputs.turnAppliedVolts = turnAppliedVolts;
		inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

		// Generate odometry samples (50Hz in sim, high frequency not needed for testing)
		inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
		inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
		inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
	}

	/**
	 * Sets drive motor to open-loop voltage.
	 *
	 * <p>Disables closed-loop control and directly sets voltage. Used for characterization and
	 * emergency stops.
	 */
	@Override
	public void setDriveOpenLoop(double output) {
		driveClosedLoop = false;
		driveAppliedVolts = output;
	}

	/**
	 * Sets turn motor to open-loop voltage.
	 *
	 * <p>Disables closed-loop control and directly sets voltage. Normally uses closed-loop position
	 * control instead.
	 */
	@Override
	public void setTurnOpenLoop(double output) {
		turnClosedLoop = false;
		turnAppliedVolts = output;
	}

	/**
	 * Sets drive motor to closed-loop velocity control.
	 *
	 * <p>Enables closed-loop mode, calculates feedforward using KV/KS, and sets PID setpoint.
	 *
	 * @param velocityRadPerSec Desired angular velocity in radians/sec
	 */
	@Override
	public void setDriveVelocity(double velocityRadPerSec) {
		driveClosedLoop = true;
		// Simple feedforward: KS * sign + KV * velocity
		driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
		driveController.setSetpoint(velocityRadPerSec);
	}

	/**
	 * Sets turn motor to closed-loop position control.
	 *
	 * <p>Enables closed-loop mode and sets PID setpoint.
	 *
	 * @param rotation Desired angle
	 */
	@Override
	public void setTurnPosition(Rotation2d rotation) {
		turnClosedLoop = true;
		turnController.setSetpoint(rotation.getRadians());
	}
}
