// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.RobotConstants;
import java.lang.reflect.Field;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Main robot class that extends AdvantageKit's LoggedRobot to enable comprehensive logging and replay.
 * 
 * <p>This class orchestrates the entire robot lifecycle, from initialization through autonomous,
 * teleop, and test modes. It configures AdvantageKit's data receivers based on the robot mode
 * (REAL, SIM, or REPLAY) and manages the WPILib CommandScheduler.
 * 
 * <p>Key responsibilities:
 * - Initialize logging infrastructure with appropriate data receivers
 * - Record build metadata for debugging and version tracking
 * - Manage the CommandScheduler lifecycle across all robot modes
 * - Handle the transition between autonomous and teleop modes
 * - Configure system properties like watchdog timeout and brownout voltage
 * 
 * <p>The three operational modes are:
 * - REAL: Logs to USB and NetworkTables, enables power distribution monitoring
 * - SIM: Publishes to NetworkTables only, runs at maximum speed without timing
 * - REPLAY: Reads from a log file and writes outputs to a new log for verification
 */
public class Robot extends LoggedRobot {
    /** Warning threshold for loop overruns (seconds). */
    private static final double loopOverrunWarningTimeout = 0.2;

    /** The autonomous command to run during autonomous period. */
    private Command m_autonomousCommand;

    /** The robot container that holds all subsystems and commands. */
    private final RobotContainer m_robotContainer;

    /**
     * Constructs the Robot and initializes logging infrastructure.
     * 
     * <p>Records build metadata (Git SHA, branch, date) and configures AdvantageKit data receivers
     * based on the current robot mode. Also adjusts the watchdog timeout to reduce spam from
     * occasional loop overruns.
     */
    @SuppressWarnings("resource")
    public Robot() {
        m_robotContainer = new RobotContainer();

        // Log build metadata for debugging and version tracking
        Logger.recordMetadata("Maven Name", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("Build Date", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("ProjectName", "SwerveDrive2025");
        Logger.recordMetadata("Robot Mode", RobotConstants.ROBOT_MODE.toString());
        Logger.recordMetadata("Git Branch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("Authors", "(WindingMotor) Isaac S & FRC 2106 Junkyard Dogs");

        // Configure logging based on robot mode
        switch (RobotConstants.ROBOT_MODE) {
            case REAL:
                // Log to USB stick ("/U/logs") and NetworkTables
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                // Enable power distribution logging for current monitoring
                new PowerDistribution(1, ModuleType.kRev);
                break;
            case SIM:
                // Run as fast as possible without timing
                setUseTiming(false);
                // Publish to NetworkTables for dashboard viewing
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                // Read from an existing log file for replay analysis
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                // Save outputs to a new log file for verification
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Start logging; must be called after all data receivers are added
        Logger.start();

        // Adjust loop overrun warning timeout to reduce spam from occasional overruns
        // The default is very aggressive; this makes it more tolerant of brief spikes
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(loopOverrunWarningTimeout);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
        }

        // Optional: Log all command lifecycle events for debugging
        // This is commented out to reduce log volume but can be enabled for troubleshooting
        /*
        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction =
                (Command command, Boolean active) -> {
                    String name = command.getName();
                    int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
                    commandCounts.put(name, count);
                    Logger.recordOutput(
                            "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
                    Logger.recordOutput("CommandsAll/" + name, count > 0);
                };
        CommandScheduler.getInstance()
                .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
        CommandScheduler.getInstance()
                .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
        CommandScheduler.getInstance()
                .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));
        */

        // Optional: Configure brownout voltage for battery protection
        // RobotController.setBrownoutVoltage(6.0);

        // Optional: Increase thread priority for more consistent loop timing
        // Threads.setCurrentThreadPriority(true, 10);
    }

    /**
     * Robot periodic method called every 20ms (by default) in all modes.
     * 
     * <p>Runs the WPILib CommandScheduler, which executes all scheduled commands, updates triggers,
     * and polls controller inputs. This is the heart of the command-based robot.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /** Called when the robot enters disabled mode. */
    @Override
    public void disabledInit() {}

    /** Called periodically while the robot is disabled. */
    @Override
    public void disabledPeriodic() {}

    /** Called when the robot exits disabled mode. */
    @Override
    public void disabledExit() {}

    /**
     * Called when autonomous mode begins.
     * 
     * <p>Retrieves the autonomous command from the robot container and schedules it.
     * The command will run until it completes or is interrupted by teleop initialization.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** Called periodically during autonomous mode. */
    @Override
    public void autonomousPeriodic() {}

    /** Called when autonomous mode ends. */
    @Override
    public void autonomousExit() {}

    /**
     * Called when teleop mode begins.
     * 
     * <p>Cancels the autonomous command if it's still running, allowing teleop commands to take over.
     */
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** Called periodically during teleop mode. */
    @Override
    public void teleopPeriodic() {}

    /** Called when teleop mode ends. */
    @Override
    public void teleopExit() {}

    /**
     * Called when test mode begins.
     * 
     * <p>Cancels all commands to provide a clean slate for testing individual subsystems.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /** Called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** Called when test mode ends. */
    @Override
    public void testExit() {}
}
