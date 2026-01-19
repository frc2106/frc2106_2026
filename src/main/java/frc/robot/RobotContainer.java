// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.coral.CMD_ElevatorCoral;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.generic.CMD_Eject;
import frc.robot.commands.generic.CMD_IntakeRace;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.constants.LIB_DriveConstants;
import frc.robot.constants.LIB_VisionConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.windingmotor.drive.Drive;
import frc.robot.lib.windingmotor.drive.gyro.*;
import frc.robot.lib.windingmotor.drive.gyro.IO_GyroBase;
import frc.robot.lib.windingmotor.drive.module.*;
import frc.robot.lib.windingmotor.vision.IO_VisionCamera;
import frc.robot.lib.windingmotor.vision.SUB_Vision;
import frc.robot.subsystems.elevator.IO_ElevatorReal;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.intake.IO_IntakeReal;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.led.SUB_Led;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.oldSUB_Superstructure;

@SuppressWarnings("unused")
public class RobotContainer {

	// ============================
	// Autonomous Configuration
	// ============================

	/** Name of the currently hard-coded PathPlanner auto to run in auto mode. */
	public static final String AUTO_NAME = "Right_3P";

	// ============================
	// Controller Configuration
	// ============================

	/** Driver joystick for translation/rotation inputs and auto-align triggers. */
	private CommandXboxController driverController;

	/** Operator joystick for scoring presets, eject, and climb sequencing. */
	private CommandXboxController operatorController;

	// ============================
	// Dashboard Choosers
	// ============================

	/**
	 * Alliance-side selector for red vs blue field transformations (unused in current logic but
	 * available).
	 */
	private SendableChooser<Boolean> isRedChooser;

	// ============================
	// Subsystems
	// ============================

	private Drive drive;

	private SUB_Intake intake;
	private SUB_Vision vision;
	private SUB_Elevator elevator;

	private oldSUB_Superstructure superstructure;

	private SUB_Led led = new SUB_Led(1, 62, AUTO_NAME);

	/** Reference to the built PathPlanner auto command for autonomousInit(). */
	public static Command AUTO_COMMAND;

	// ============================
	// Construction and Initialization
	// ============================

	public RobotContainer() {
		// Initialize Robot Components
		initializeControllers();
		initializeSubsystems();
		configurePathplannerCommands();
		configureButtonBindings();

		// Add alliance selector to SmartDashboard
		isRedChooser = new SendableChooser<Boolean>();
		isRedChooser.addOption("Red", true);
		isRedChooser.addOption("Blue", false);
		isRedChooser.setDefaultOption("Red", true);
		SmartDashboard.putData("Alliance", isRedChooser);

		// Create and cache the PathPlanner auto command
		AUTO_COMMAND = AutoBuilder.buildAuto(AUTO_NAME);

		// Start USB camera feed for driver visibility
		CameraServer.startAutomaticCapture();

		// Optional: could start a second camera here for alignment
		// CameraServer.startAutomaticCapture();
		// alignmentCamera = new AlignmentCamera(0, "Driver CAM");
	}

	/**
	 * Initialize controller objects with USB port numbers. Port 0 = driver, Port 1 = operator per FRC
	 * convention.
	 */
	private void initializeControllers() {
		driverController = new CommandXboxController(0);
		operatorController = new CommandXboxController(1);
	}

	/**
	 * Initialize all subsystems with appropriate IO implementations based on
	 * RobotConstants.ROBOT_MODE.
	 *
	 * <p>Three modes are supported: - REAL: Hardware IO implementations that talk to actual motor
	 * controllers, sensors, and gyro - SIM: Physics simulation IO implementations that model
	 * mechanism behavior for testing without hardware - REPLAY: "stub" IO implementations that read
	 * from logs; used with AdvantageKit replay to debug using real data
	 *
	 * <p>The Switch construct is the standard pattern for selecting IO variants while keeping
	 * subsystem logic identical.
	 */
	private void initializeSubsystems() {
		// Intake and elevator always use real IO (they could be extended with sim variants later)
		intake = new SUB_Intake(new IO_IntakeReal());
		elevator = new SUB_Elevator(new IO_ElevatorReal());
		// climb = new SUB_Climb(new IO_ClimbReal());

		// Initialize CANand event loop (used by some sensor implementations)
		CanandEventLoop.getInstance();

		// Drive subsystem: IO varies dramatically by mode
		switch (RobotConstants.ROBOT_MODE) {
			case REAL:
				// Real robot, instantiate hardware IO implementations
				drive =
						new Drive(
								new IO_GyroReal(),
								new IO_ModuleReal(LIB_DriveConstants.FrontLeft),
								new IO_ModuleReal(LIB_DriveConstants.FrontRight),
								new IO_ModuleReal(LIB_DriveConstants.BackLeft),
								new IO_ModuleReal(LIB_DriveConstants.BackRight));
				break;

			case SIM:
				// Sim robot, instantiate physics sim IO implementations
				drive =
						new Drive(
								new IO_GyroBase() {},
								new IO_ModuleSim(LIB_DriveConstants.FrontLeft),
								new IO_ModuleSim(LIB_DriveConstants.FrontRight),
								new IO_ModuleSim(LIB_DriveConstants.BackLeft),
								new IO_ModuleSim(LIB_DriveConstants.BackRight));
				break;

			default:
				// Replayed robot, disable IO implementations
				drive =
						new Drive(
								new IO_GyroBase() {},
								new IO_ModuleBase() {},
								new IO_ModuleBase() {},
								new IO_ModuleBase() {},
								new IO_ModuleBase() {});
				break;
		}

		// Vision subsystem: cameras feed pose measurements to drive
		vision =
				new SUB_Vision(
						drive::addVisionMeasurement,
						new IO_VisionCamera(
								LIB_VisionConstants.camera0Name, LIB_VisionConstants.robotToCamera0),
						new IO_VisionCamera(
								LIB_VisionConstants.camera1Name, LIB_VisionConstants.robotToCamera1));

		// Superstructure binds all mechanisms together
		superstructure = new oldSUB_Superstructure(drive, intake, elevator, led, operatorController);

		// Setup Sendable Choosers
		isRedChooser = new SendableChooser<Boolean>();

		// Set up SysId routines (commented out but kept as examples)
		/*
		autoChooser.addOption(
						"Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
		autoChooser.addOption(
						"Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
		autoChooser.addOption(
						"Drive SysId (Quasistatic Forward)",
						drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
						"Drive SysId (Quasistatic Reverse)",
						drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		autoChooser.addOption(
						"Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
						"Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
						*/
	}

	/**
	 * Register all NamedCommands that PathPlanner autonomous routines can reference.
	 *
	 * <p>Each command typically wraps a CMD_Superstructure state change, allowing the PP path to
	 * coordinate scoring actions at specific waypoints. The names here must match exactly what is
	 * used in the PP GUI.
	 */
	private void configurePathplannerCommands() {
		NamedCommands.registerCommand(
				"Intake_Coral", new CMD_Superstructure(superstructure, SuperstructureState.CORAL_STATION));

		NamedCommands.registerCommand(
				"Intake_Race",
				new CMD_IntakeRace(intake)
						.andThen(new CMD_Superstructure(superstructure, SuperstructureState.IDLE)));

		NamedCommands.registerCommand(
				"L1", new CMD_Superstructure(superstructure, SuperstructureState.L1_SCORING));
		NamedCommands.registerCommand(
				"L2", new CMD_Superstructure(superstructure, SuperstructureState.L2_SCORING));
		NamedCommands.registerCommand(
				"L2C", new CMD_Superstructure(superstructure, SuperstructureState.L2_CLEAR));
		NamedCommands.registerCommand(
				"L3", new CMD_Superstructure(superstructure, SuperstructureState.L3_SCORING));
		NamedCommands.registerCommand(
				"L3C", new CMD_Superstructure(superstructure, SuperstructureState.L3_CLEAR));
		NamedCommands.registerCommand(
				"L4", new CMD_Superstructure(superstructure, SuperstructureState.L4_SCORING_AUTO));
		NamedCommands.registerCommand(
				"L4C", new CMD_Superstructure(superstructure, SuperstructureState.L4_CLEAR));

		NamedCommands.registerCommand("Eject", new CMD_Eject(superstructure));

		NamedCommands.registerCommand(
				"Idle", new CMD_Superstructure(superstructure, SuperstructureState.IDLE));

		NamedCommands.registerCommand(
				"ALGAE_L2", new CMD_Superstructure(superstructure, SuperstructureState.ALGAE_L2));
		NamedCommands.registerCommand(
				"ALGAE_L3", new CMD_Superstructure(superstructure, SuperstructureState.ALGAE_L3));
		NamedCommands.registerCommand(
				"ALGAE_BARGE", new CMD_Superstructure(superstructure, SuperstructureState.ALGAE_BARGE));
		NamedCommands.registerCommand(
				"ALGAE_PROCESSOR",
				new CMD_Superstructure(superstructure, SuperstructureState.ALGAE_PROCESSOR));
	}

	/**
	 * Bind input axes and buttons to commands using CommandXboxController's fluent API.
	 *
	 * <p>Includes: - Default drive command with rotation assist - Coral scoring preset buttons (L4,
	 * L3, etc.) - Dynamic algae handling - Intake/eject controls - Auto-align triggers with
	 * debouncing - Climb sequence activation
	 */
	private void configureButtonBindings() {
		// Drive w/ Assist Rotation: default command runs continuously unless interrupted
		drive.setDefaultCommand(
				DriveCommands.driveWithAssist(
						drive,
						() -> -driverController.getRawAxis(1),
						() -> driverController.getRawAxis(0),
						() -> -driverController.getRawAxis(3),
						() -> driverController.button(3).getAsBoolean()));

		// Coral scoring presets
		operatorController.rightBumper().onTrue(new CMD_ElevatorCoral(superstructure, true));
		operatorController.leftBumper().onTrue(new CMD_ElevatorCoral(superstructure, false));
		operatorController
				.rightTrigger()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.L4_SCORING_TELE));
		operatorController
				.leftTrigger()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.L3_SCORING));

		// Algae handling
		operatorController.y().onTrue(superstructure.dynamicAlage());
		operatorController
				.povUp()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.ALGAE_BARGE));
		operatorController
				.povDown()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.ALGAE_GROUND));

		// Eject and intake
		operatorController.x().onTrue(new CMD_Eject(superstructure));
		operatorController
				.a()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.CORAL_STATION));
		operatorController.b().onTrue(new CMD_Superstructure(superstructure, SuperstructureState.IDLE));

		// First Auto Align: triggers on button press; debounced to prevent chatter
		driverController
				.button(1)
				.onChange(
						DriveCommands.driveAlign(
								drive,
								() -> oldSUB_Superstructure.globalFirstPose,
								driverController,
								elevator.getHeight()))
				.debounce(.1, DebounceType.kBoth);

		// Second Auto Align
		driverController
				.button(4)
				.onChange(
						DriveCommands.driveAlign(
								drive,
								() -> oldSUB_Superstructure.globalSecondPose,
								driverController,
								elevator.getHeight()))
				.debounce(.1, DebounceType.kBoth);

		// Climb Automatic: operator confirmation looped into the sequence itself
		/* operatorController
				.povRight()
				.onTrue(
						climb.climbSequence(
								() -> operatorController.povRight().getAsBoolean(), 1.0, led, superstructure));
		// Climb zero/reset
		operatorController.povLeft().onTrue(climb.goToPosition(0, 1)); */
	}

	/**
	 * Return the command to run in autonomous mode.
	 *
	 * <p>Uses the cached PathPlanner auto command built at construction time. If no auto is
	 * available, prints a warning to the console for debugging.
	 */
	public Command getAutonomousCommand() {
		if (AUTO_COMMAND != null) {
			return AUTO_COMMAND;
		} else {
			return new PrintCommand("Auto Command is NULL!");
		}
	}
}
