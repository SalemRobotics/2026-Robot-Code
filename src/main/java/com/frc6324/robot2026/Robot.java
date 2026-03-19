/*
 * Copyright (c) 2025 The Blue Devils.
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */
package com.frc6324.robot2026;

import com.ctre.phoenix6.SignalLogger;
import com.frc6324.lib.util.LoggedTracer;
import com.frc6324.lib.util.PhoenixUtil;
import com.frc6324.lib.util.VirtualSubsystem;
import com.frc6324.robot2026.sim.MapleSimManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Optional;
import lombok.val;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  public static Alliance autoWinner = Alliance.Blue;
  private static LoggedDashboardChooser<Alliance> autoWinnerOverride;

  private Command autonomousCommand = null;
  private final RobotContainer robotContainer;
  private final Runtime runtime = Runtime.getRuntime();

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data recievers and replay source
    switch (Constants.CURRENT_MODE) {
      case REAL -> {
        // Real robot, publish to logs and NT
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> {
        // Sim, only publish to NT
        Logger.addDataReceiver(new NT4Publisher());
      }
      case REPLAY -> {
        // Disable timing to run as fast as possible
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();

        // Replaing a log, set up replay source and log to a file
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    // Silence warnings for disconnected joysticks
    DriverStation.silenceJoystickConnectionWarning(true);
    SignalLogger.enableAutoLogging(false);

    // Start the akit logger
    Logger.start();

    autoWinnerOverride = new LoggedDashboardChooser<>("Auto Winner");
    autoWinnerOverride.addDefaultOption("Blue", Alliance.Blue);
    autoWinnerOverride.addOption("Red", Alliance.Red);

    // Initialize the robot container
    robotContainer = new RobotContainer();
    PhoenixUtil.synchronizeSignals(2);
  }

  /**
   * Gets if the current alliance has an active hub.
   *
   * @return Whether the current alliance's hub is active.
   */
  public static boolean getIsAllianceActive() {
    // If we're in autonomous, both hubs are active
    if (DriverStation.isAutonomous()) {
      return true;
    }

    final Optional<Alliance> selfAllianceOpt = DriverStation.getAlliance();
    if (selfAllianceOpt.isEmpty()) {
      return false;
    }

    final Alliance selfAlliance = selfAllianceOpt.get();
    final double teleopTime = DriverStation.getMatchTime();

    // In the first 10 secs of teleop and the last 30 secs, both hubs are active
    if (teleopTime >= 130 || teleopTime <= 30) {
      return true;
    } else if ((teleopTime <= 105 && teleopTime >= 80) || (teleopTime <= 55 && teleopTime >= 30)) {
      return autoWinner == selfAlliance;
    } else {
      return autoWinner != selfAlliance;
    }
  }

  /**
   * Gets the time left in the current phase.
   *
   * @return The time left for the current active hub.
   */
  public static double getTimeLeftInPhase() {
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Match/Phase", "Not tele :/");
      return 0;
    }

    final double teleopTime = DriverStation.getMatchTime();

    if (teleopTime >= 130.0) {
      Logger.recordOutput("Match/Phase", "After Auto");
      return teleopTime - 130;
    } else if (teleopTime <= 30.0) {
      Logger.recordOutput("Match/Phase", "Endgame");
      return teleopTime;
    } else {
      Logger.recordOutput("Match/Phase", "One Hub");
      return (teleopTime - 30) % 25;
    }
  }

  @Override
  public void robotPeriodic() {
    Logger.runEveryN(
        5,
        () -> {
          val totalMemory = runtime.totalMemory();
          val usedMemory = totalMemory - runtime.freeMemory();
          val utilization = ((double) (usedMemory)) / ((double) (totalMemory));

          Logger.recordOutput("Robot/Used Memory %", utilization * 100);
        });
    Logger.recordOutput("Match/Hub Active", getIsAllianceActive());
    Logger.recordOutput("Match/Phase Time", getTimeLeftInPhase());

    // Set thread to highest priority to improve performance
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the command scheduler

    LoggedTracer.reset();
    PhoenixUtil.refreshAllSignals();
    LoggedTracer.record("Phoenix Refresh");

    VirtualSubsystem.allPeriodics();
    LoggedTracer.record("Virtual Subsystems");

    CommandScheduler.getInstance().run();
    LoggedTracer.record("Command Scheduler");

    // Set the thread to low priority to let other things run (such as NT)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Get the new autonomous command that is selected
    autonomousCommand = robotContainer.getAutonomousCommand();

    // Schedule the auto command
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    autoWinner = Alliance.Blue;
  }

  @Override
  public void teleopInit() {
    // If an auto command is running, cancel it
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    if (DriverStation.isFMSAttached()) {
      switch (DriverStation.getGameSpecificMessage()) {
        case "B" -> autoWinner = Alliance.Blue;
        case "R" -> autoWinner = Alliance.Red;
        default -> autoWinner = null;
      }
    }

    if (autoWinner == null) {
      autoWinner = autoWinnerOverride.get();
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    // Cancel all scheduled commands during test
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    MapleSimManager.getInstance().simulationPeriodic();
  }
}
