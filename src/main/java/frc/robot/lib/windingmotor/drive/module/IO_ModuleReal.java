// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.windingmotor.drive.module;

import static frc.robot.lib.windingmotor.util.phoenix.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.windingmotor.drive.Drive;
import frc.robot.lib.windingmotor.util.phoenix.PhoenixOdometryThread;
import java.util.Queue;

/**
 * Real hardware implementation for swerve module IO.
 *
 * <p>Controls a complete swerve module with: - TalonFX drive motor (velocity control for wheel
 * speed) - TalonFX turn motor (position control for wheel angle) - Canandmag absolute encoder (for
 * zeroing turn position)
 *
 * <p>Phoenix 6 provides multiple closed-loop output options: - VoltageOut: Direct voltage control
 * (open loop) - VelocityVoltage: Velocity control with voltage output - PositionVoltage: Position
 * control with voltage output - TorqueCurrentFOC: Torque control with Field-Oriented Control (FOC)
 *
 * <p>The implementation uses TunerConstants for configuration but allows local overrides for
 * behaviors not exposed by the constants (e.g., current limits, neutral mode).
 */
public class IO_ModuleReal implements IO_ModuleBase {
	/** Swerve module constants from Phoenix Tuner (motor IDs, gains, gear ratios). */
	private final SwerveModuleConstants<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			constants;

	// Hardware objects
	private final TalonFX driveTalon;
	private final TalonFX turnTalon;
	private final CANcoder cancoder;

	// Control requests (created once, reused to avoid garbage collection)
	/** Voltage control request for open-loop commands. */
	private final VoltageOut voltageRequest = new VoltageOut(0);

	/** Position control request using voltage output. */
	private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

	/** Velocity control request using voltage output. */
	private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

	/** Torque control request using FOC (Field-Oriented Control). */
	private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);

	/** Position control request using torque (FOC). */
	private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
			new PositionTorqueCurrentFOC(0.0);

	/** Velocity control request using torque (FOC). */
	private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
			new VelocityTorqueCurrentFOC(0.0);

	// Timestamp queue for odometry synchronization
	private final Queue<Double> timestampQueue;

	// Drive motor signals
	private final StatusSignal<Angle> drivePosition;
	private final Queue<Double> drivePositionQueue;
	private final StatusSignal<AngularVelocity> driveVelocity;
	private final StatusSignal<Voltage> driveAppliedVolts;
	private final StatusSignal<Current> driveCurrent;

	// Turn motor signals
	private final StatusSignal<Angle> turnPosition;
	private final Queue<Double> turnPositionQueue;
	private final StatusSignal<AngularVelocity> turnVelocity;
	private final StatusSignal<Voltage> turnAppliedVolts;
	private final StatusSignal<Current> turnCurrent;

	// Connection debouncers (prevent false positives from brief CAN hiccups)
	private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
	private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
	private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

	/**
	 * Constructs and configures a real swerve module.
	 *
	 * <p>Configuration process: 1. Configure drive motor: brake mode, gains, current limits, gear
	 * ratio, inversion 2. Configure turn motor: brake mode, gains, continuous wrap, gear ratio,
	 * inversion 3. Configure Canandmag: zero offset, direction inversion 4. Zero turn motor to
	 * absolute encoder (critical for accurate odometry) 5. Create odometry queues for high-frequency
	 * sampling 6. Configure signal update frequencies (250Hz for odometry, 50Hz for telemetry) 7.
	 * Optimize CAN bus utilization
	 *
	 * @param constants Module constants from Phoenix Tuner (including motor IDs, gains, etc.)
	 */
	public IO_ModuleReal(
			SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
					constants) {
		this.constants = constants;
		driveTalon = new TalonFX(constants.DriveMotorId, RobotConstants.CANBUS_CANIVORE);
		turnTalon = new TalonFX(constants.SteerMotorId, RobotConstants.CANBUS_CANIVORE);
		cancoder = new CANcoder(constants.EncoderId);

		// Configure drive motor
		var driveConfig = constants.DriveMotorInitialConfigs;
		driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Brake when neutral
		driveConfig.Slot0 = constants.DriveMotorGains; // PID gains from Tuner
		driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio; // Gear ratio
		driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent; // Slip limit
		driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
		driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent; // Current limit
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		driveConfig.MotorOutput.Inverted =
				constants.DriveMotorInverted
						? InvertedValue.Clockwise_Positive
						: InvertedValue.CounterClockwise_Positive;
		// Retry configuration up to 5 times (handles transient CAN errors)
		tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
		// Reset position to zero (relative encoder)
		tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

		// Configure turn motor
		var turnConfig = new TalonFXConfiguration();
		turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		turnConfig.Slot0 = constants.SteerMotorGains;

		turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
		turnConfig.Feedback.RotorToSensorRatio = 1.0;
		turnConfig.Feedback.SensorToMechanismRatio = 1.0;

		// turnConfig.Feedback.SensorToMechanismRatio = LIB_DriveConstants.kSteerGearRatio;

		// Motion Magic settings for smooth turning
		turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
		turnConfig.MotionMagic.MotionMagicAcceleration =
				turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
		turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
		turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
		turnConfig.ClosedLoopGeneral.ContinuousWrap = true; // Wrap at ±180°
		turnConfig.MotorOutput.Inverted =
				constants.SteerMotorInverted
						? InvertedValue.Clockwise_Positive
						: InvertedValue.CounterClockwise_Positive;
		tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

		// Configure Canandmag absolute encoder
		/*
		CanandmagSettings canandmagSettings = new CanandmagSettings();
		canandmagSettings.setZeroOffset(constants.EncoderOffset); // Zero offset from Tuner
		canandmagSettings.setInvertDirection(true); // Invert direction to match motor orientation
		canandmag.setSettings(canandmagSettings);
		*/

		CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
		encoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
		if (constants.EncoderInverted) {
			encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
		} else {
			encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		}
		cancoder.getConfigurator().apply(encoderConfig);

		// Verify Canandmag connection
		// if (!canandmag.isConnected()) {
		//	throw new RuntimeException("Canandmag not connected during initialization");
		// }

		// Get absolute position (0 to 1 rotation)
		double absolutePosition = cancoder.getAbsolutePosition().getValueAsDouble();

		// Zero turn motor to absolute encoder
		// Canandmag returns 0-1, TalonFX expects rotations (no conversion needed in Phoenix v6)
		tryUntilOk(10, () -> turnTalon.setPosition(absolutePosition, 0.25));

		// Create timestamp queue for odometry synchronization
		timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

		// Create drive status signals
		drivePosition = driveTalon.getPosition();
		drivePositionQueue =
				PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
		driveVelocity = driveTalon.getVelocity();
		driveAppliedVolts = driveTalon.getMotorVoltage();
		driveCurrent = driveTalon.getStatorCurrent();

		// Create turn status signals
		turnPosition = turnTalon.getPosition();
		turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition());
		turnVelocity = turnTalon.getVelocity();
		turnAppliedVolts = turnTalon.getMotorVoltage();
		turnCurrent = turnTalon.getStatorCurrent();

		// Configure signal update frequencies
		// Odometry signals at high frequency (250Hz on CAN FD)
		BaseStatusSignal.setUpdateFrequencyForAll(
				Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
		// Telemetry signals at standard frequency (50Hz)
		BaseStatusSignal.setUpdateFrequencyForAll(
				50.0,
				driveVelocity,
				driveAppliedVolts,
				driveCurrent,
				turnVelocity,
				turnAppliedVolts,
				turnCurrent);
		// Minimize CAN bus bandwidth
		ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
	}

	/**
	 * Updates the inputs object with latest hardware data.
	 *
	 * <p>Process: 1. Bulk refresh all signals (efficient CAN bus usage) 2. Update drive inputs with
	 * debounced connection status 3. Update turn inputs with debounced connection status 4. Drain
	 * odometry queues into arrays 5. Clear queues to prevent memory growth
	 *
	 * <p>Unit conversions: - Phoenix native: rotations - WPILib: radians - Drive position is
	 * multiplied by wheel radius elsewhere for linear distance
	 */
	@Override
	public void updateInputs(ModuleInputs inputs) {
		// Refresh all drive signals
		var driveStatus =
				BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
		// Refresh all turn signals
		var turnStatus =
				BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);

		// Update drive inputs with debounced connection status
		inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
		inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
		inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
		inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
		inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

		// Update turn inputs with debounced connection status
		inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
		inputs.turnEncoderConnected = cancoder.isConnected();
		inputs.turnAbsolutePosition =
				Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble());
		inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
		inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
		inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
		inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

		// Drain odometry queues into arrays
		inputs.odometryTimestamps =
				timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
		inputs.odometryDrivePositionsRad =
				drivePositionQueue.stream()
						.mapToDouble((Double value) -> Units.rotationsToRadians(value))
						.toArray();
		inputs.odometryTurnPositions =
				turnPositionQueue.stream()
						.map((Double value) -> Rotation2d.fromRotations(value))
						.toArray(Rotation2d[]::new);

		// CRITICAL: Clear queues to prevent unbounded memory growth
		// These queues accumulate at 250Hz, so must be drained every cycle
		timestampQueue.clear();
		drivePositionQueue.clear();
		turnPositionQueue.clear();
	}

	/**
	 * Runs drive motor in open-loop voltage control.
	 *
	 * <p>Used for: - Characterization tests - Emergency stop (zero voltage) - Direct driver control
	 * in test mode
	 *
	 * <p>The output type (Voltage vs TorqueCurrentFOC) is selected based on TunerConstants.
	 */
	@Override
	public void setDriveOpenLoop(double output) {
		driveTalon.setControl(
				switch (constants.DriveMotorClosedLoopOutput) {
					case Voltage -> voltageRequest.withOutput(output);
					case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
				});
	}

	/**
	 * Runs turn motor in open-loop voltage control.
	 *
	 * <p>Typically only used for characterization or emergency stops. Normal operation uses
	 * closed-loop position control via setTurnPosition().
	 */
	@Override
	public void setTurnOpenLoop(double output) {
		turnTalon.setControl(
				switch (constants.SteerMotorClosedLoopOutput) {
					case Voltage -> voltageRequest.withOutput(output);
					case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
				});
	}

	/**
	 * Runs drive motor in closed-loop velocity control.
	 *
	 * <p>Converts radians/sec to rotations/sec (Phoenix native units) and applies the appropriate
	 * control request based on TunerConstants.
	 *
	 * @param velocityRadPerSec Desired wheel angular velocity in radians/sec
	 */
	@Override
	public void setDriveVelocity(double velocityRadPerSec) {
		double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
		driveTalon.setControl(
				switch (constants.DriveMotorClosedLoopOutput) {
					case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
					case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
				});
	}

	/**
	 * Runs turn motor in closed-loop position control.
	 *
	 * <p>Uses Motion Magic for smooth, controlled motion with expo-kV and expo-kA for velocity and
	 * acceleration feedforward. Continuous wrap is enabled to handle the ±180° boundary seamlessly.
	 *
	 * @param rotation Desired wheel angle
	 */
	@Override
	public void setTurnPosition(Rotation2d rotation) {
		turnTalon.setControl(
				switch (constants.SteerMotorClosedLoopOutput) {
					case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
					case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(
							rotation.getRotations());
				});
	}
}
