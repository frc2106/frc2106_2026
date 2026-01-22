// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.constants.LIB_DriveConstants;
import frc.robot.constants.LIB_VisionConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.windingmotor.drive.Drive;
import frc.robot.lib.windingmotor.drive.gyro.*;
import frc.robot.lib.windingmotor.drive.module.*;
import frc.robot.lib.windingmotor.vision.IO_VisionCamera;
import frc.robot.lib.windingmotor.vision.SUB_Vision;
import frc.robot.subsystems.indexer.IO_IndexerReal;
import frc.robot.subsystems.indexer.SUB_Indexer;
import frc.robot.subsystems.intake.IO_IntakeReal;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.led.SUB_Led;
import frc.robot.subsystems.shooter.IO_ShooterReal;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.superstructure.SUB_Superstructure;

@SuppressWarnings("unused")
public class RobotContainer {

	// The auto to run
	public static final String AUTO_NAME = "LINE";
	public static Command AUTO_COMMAND;

	// Controllers
	private CommandXboxController driverController;
	private CommandXboxController operatorController;

	// Are we on red???
	private SendableChooser<Boolean> isRedChooser;

	// Subsystems
	private SUB_Indexer indexer;
	private SUB_Intake intake;
	private SUB_Led led = new SUB_Led(1, 62, AUTO_NAME);
	private SUB_Shooter shooter;

	// Libary Subsystems
	private Drive drive;
	private SUB_Vision vision;

	// The superstructure
	private SUB_Superstructure superstructure;

	public RobotContainer() {
		// Initialize Robot Components
		initializeControllers();
		initializeSubsystems();
		// configurePathplannerCommands();
		// configureButtonBindings();

		// Add alliance selector to SmartDashboard
		isRedChooser = new SendableChooser<Boolean>();
		isRedChooser.addOption("Red", true);
		isRedChooser.addOption("Blue", false);
		isRedChooser.setDefaultOption("Red", true);
		SmartDashboard.putData("Alliance", isRedChooser);

		// Create and cache the PathPlanner auto command
		AUTO_COMMAND = AutoBuilder.buildAuto(AUTO_NAME);

		drive.setDefaultCommand(
				DriveCommands.driveNormal(
						drive,
						() -> driverController.getRawAxis(1),
						() -> -driverController.getRawAxis(0),
						() -> -driverController.getRawAxis(3)));

		operatorController
				.a()
				.onTrue(new CMD_Superstructure(superstructure, SUB_Superstructure.RobotState.INTAKING));

		operatorController
				.x()
				.onTrue(new CMD_Superstructure(superstructure, SUB_Superstructure.RobotState.SHOOTING));

		operatorController
				.b()
				.onTrue(new CMD_Superstructure(superstructure, SUB_Superstructure.RobotState.IDLE));

		// drive.setDefaultCommand(DriveCommands.driveTest(drive));
	}

	private void initializeControllers() {
		driverController = new CommandXboxController(0);
		operatorController = new CommandXboxController(3);
	}

	private void initializeSubsystems() {

		indexer =
				new SUB_Indexer(
						new IO_IndexerReal(
								RobotConstants.Indexer.KICKER_MOTOR_CONFIG,
								RobotConstants.Indexer.SPINNER_MOTOR_CONFIG));

		intake =
				new SUB_Intake(
						new IO_IntakeReal(
								RobotConstants.Intake.INTAKE_MOTOR_CONFIG,
								RobotConstants.Intake.SLIDER_MOTOR_CONFIG));

		shooter =
				new SUB_Shooter(
						new IO_ShooterReal(
								RobotConstants.Shooter.SHOOTER_MOTOR_ONE_CONFIG,
								RobotConstants.Shooter.SHOOTER_MOTOR_TWO_CONFIG,
								RobotConstants.Shooter.TURRET_MOTOR_CONFIG));

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
								LIB_VisionConstants.camera1Name, LIB_VisionConstants.robotToCamera1)
								);

		// Superstructure binds all mechanisms together
		superstructure =
				new SUB_Superstructure(indexer, intake, led, shooter, drive, vision, driverController);

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

	/*
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
				*/

	/*
	private void configureButtonBindings() {
		// Drive w/ Assist Rotation: default command runs continuously unless interrupted


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
		operatorController
				.povRight()
				.onTrue(
						climb.climbSequence(
								() -> operatorController.povRight().getAsBoolean(), 1.0, led, superstructure));
		// Climb zero/reset
		operatorController.povLeft().onTrue(climb.goToPosition(0, 1));
	}
	*/

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
