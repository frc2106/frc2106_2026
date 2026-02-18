// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.windingmotor.util.fieldsim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.LIB_DriveConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.windingmotor.drive.Drive;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.shooter.SUB_Shooter;
import org.littletonrobotics.junction.Logger;

public class SimulationManager {

	private static SimulationManager instance;

	public static SimulationManager getInstance() {
		if (instance == null) instance = new SimulationManager();
		return instance;
	}

	private final FuelSim fuelSim;

	// -----------------------------------------------------------------------
	// Robot physical dimensions derived from LIB_DriveConstants
	// Module positions are measured to the wheel center from robot center.
	// Full frame width/length = 2 * outermost module position + bumper thickness.
	// Bumper thickness is typically ~3.25 inches (0.083m) per side.
	// -----------------------------------------------------------------------
	private static final double BUMPER_THICKNESS_M = 0.083; // ~3.25 inches per side

	// Outermost module Y position = 11.375 in = 0.289 m
	private static final double ROBOT_WIDTH_M =
			2 * (LIB_DriveConstants.FrontLeft.LocationY + BUMPER_THICKNESS_M);

	// Outermost module X position = 10.375 in = 0.264 m
	private static final double ROBOT_LENGTH_M =
			2 * (LIB_DriveConstants.FrontLeft.LocationX + BUMPER_THICKNESS_M);

	// Standard FRC bumper height: bumpers sit 2–7.5 inches off floor, top at ~7.5 in = 0.191 m
	// Using 0.216 m (~8.5 in) to be slightly conservative
	private static final double BUMPER_HEIGHT_M = 0.216;

	// -----------------------------------------------------------------------
	// Intake bounding box in robot-relative coordinates (robot center = 0,0)
	// Intake extends from the front face of the robot outward.
	// Front face X = ROBOT_LENGTH_M / 2 (before bumpers, relative to center)
	// -----------------------------------------------------------------------
	private static final double INTAKE_DEPTH_M = 0.15; // how far intake extends beyond frame
	private static final double INTAKE_OVERLAP_M = 0.10; // overlap inside frame for reliable pickup
	private static final double INTAKE_WIDTH_M = 0.70; // intake width — tune to your actual intake

	private static final double INTAKE_X_MIN = ROBOT_LENGTH_M / 2 - INTAKE_OVERLAP_M;
	private static final double INTAKE_X_MAX = ROBOT_LENGTH_M / 2 + INTAKE_DEPTH_M;
	private static final double INTAKE_Y_MIN = -INTAKE_WIDTH_M / 2;
	private static final double INTAKE_Y_MAX = INTAKE_WIDTH_M / 2;

	// -----------------------------------------------------------------------
	// Shooter launch height — turret is offset from robot center, use Z estimate
	// No Z offset in RobotConstants so we estimate from robot structure.
	// Tune this to the actual height of your shooter exit point from the floor.
	// -----------------------------------------------------------------------
	private static final double SHOOTER_LAUNCH_HEIGHT_M = 0.55; // ~21.6 inches — measure your robot

	// Logging state
	private int totalFuelIntaked = 0;
	private int totalFuelLaunched = 0;
	private int blueHubScore = 0;
	private int redHubScore = 0;
	private double lastLaunchExitVelocity = 0.0;
	private double lastLaunchTurretAngleDeg = 0.0;
	private boolean intakeRegistered = false;

	private SimulationManager() {
		fuelSim = new FuelSim("FieldSimulation");
		fuelSim.spawnStartingFuel();
		fuelSim.start();

		Logger.recordOutput("SimulationManager/Status", "Initialized");
	}

	/**
	 * Register the robot and intake with FuelSim. Call once from RobotContainer after subsystems are
	 * initialized.
	 */
	public void registerRobot(Drive drive, SUB_Intake intake, SUB_Shooter shooter) {
		fuelSim.registerRobot(
				ROBOT_WIDTH_M, ROBOT_LENGTH_M, BUMPER_HEIGHT_M, drive::getPose, drive::getChassisSpeeds);

		fuelSim.registerIntake(
				INTAKE_X_MIN,
				INTAKE_X_MAX,
				INTAKE_Y_MIN,
				INTAKE_Y_MAX,
				() -> intake.isIntakeRunning(),
				() -> {
					totalFuelIntaked++;
					Logger.recordOutput("SimulationManager/TotalFuelIntaked", totalFuelIntaked);
				});

		intakeRegistered = true;
		Logger.recordOutput("SimulationManager/Status", "Robot Registered");
		Logger.recordOutput("SimulationManager/IntakeRegistered", intakeRegistered);
	}

	/**
	 * Launch a fuel projectile from the turret. Uses shooter constants from RobotConstants for hood
	 * angle and efficiency.
	 *
	 * @param exitVelocityMPS Exit velocity in m/s
	 * @param turretAngleRad Robot-relative turret angle in radians
	 */
	public void launchFuel(double exitVelocityMPS, double turretAngleRad) {
		LinearVelocity velocity = MetersPerSecond.of(exitVelocityMPS);
		Angle hoodAngle = Radians.of(RobotConstants.Shooter.SHOOTER_ANGLE_RADIANS);
		Angle turretYaw = Radians.of(turretAngleRad);
		Distance launchHeight = Meters.of(SHOOTER_LAUNCH_HEIGHT_M);

		fuelSim.launchFuel(velocity, hoodAngle, turretYaw, launchHeight);

		totalFuelLaunched++;
		lastLaunchExitVelocity = exitVelocityMPS;
		lastLaunchTurretAngleDeg = Math.toDegrees(turretAngleRad);

		Logger.recordOutput("SimulationManager/TotalFuelLaunched", totalFuelLaunched);
		Logger.recordOutput("SimulationManager/LastLaunchExitVelocityMPS", lastLaunchExitVelocity);
		Logger.recordOutput("SimulationManager/LastLaunchTurretAngleDeg", lastLaunchTurretAngleDeg);
		Logger.recordOutput(
				"SimulationManager/LastLaunchHoodAngleDeg",
				Math.toDegrees(RobotConstants.Shooter.SHOOTER_ANGLE_RADIANS));
	}

	/** Called from Robot.simulationPeriodic() */
	public void update() {
		fuelSim.updateSim();

		blueHubScore = FuelSim.Hub.BLUE_HUB.getScore();
		redHubScore = FuelSim.Hub.RED_HUB.getScore();

		Logger.recordOutput("SimulationManager/BlueHubScore", blueHubScore);
		Logger.recordOutput("SimulationManager/RedHubScore", redHubScore);
		Logger.recordOutput("SimulationManager/TotalFuelIntaked", totalFuelIntaked);
		Logger.recordOutput("SimulationManager/TotalFuelLaunched", totalFuelLaunched);
		Logger.recordOutput("SimulationManager/IntakeActive", intakeRegistered);
	}

	/** Reset field fuel and all scores */
	public void resetFuel() {
		fuelSim.clearFuel();
		fuelSim.spawnStartingFuel();

		FuelSim.Hub.BLUE_HUB.resetScore();
		FuelSim.Hub.RED_HUB.resetScore();

		totalFuelIntaked = 0;
		totalFuelLaunched = 0;
		blueHubScore = 0;
		redHubScore = 0;

		Logger.recordOutput("SimulationManager/Status", "Fuel Reset");
		Logger.recordOutput("SimulationManager/BlueHubScore", 0);
		Logger.recordOutput("SimulationManager/RedHubScore", 0);
		Logger.recordOutput("SimulationManager/TotalFuelIntaked", 0);
		Logger.recordOutput("SimulationManager/TotalFuelLaunched", 0);
	}
}
