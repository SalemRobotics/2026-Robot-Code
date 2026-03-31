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

import static com.frc6324.robot2026.Constants.*;

import com.frc6324.lib.util.FieldConstants.LinesVertical;
import com.frc6324.lib.util.IOLayer;
import com.frc6324.lib.util.PoseExtensions;
import com.frc6324.robot2026.commands.AutoCommands;
import com.frc6324.robot2026.commands.AutoCommands.AllianceSide;
import com.frc6324.robot2026.commands.DriveCommands;
import com.frc6324.robot2026.commands.ShooterCommands;
import com.frc6324.robot2026.commands.ShooterCommands.*;
import com.frc6324.robot2026.subsystems.drive.*;
import com.frc6324.robot2026.subsystems.drive.DriveIO.DriveIOReplay;
import com.frc6324.robot2026.subsystems.indexer.*;
import com.frc6324.robot2026.subsystems.intake.*;
import com.frc6324.robot2026.subsystems.leds.LEDs;
import com.frc6324.robot2026.subsystems.leds.LEDs.LEDState;
import com.frc6324.robot2026.subsystems.rollers.*;
import com.frc6324.robot2026.subsystems.shooter.*;
import com.frc6324.robot2026.subsystems.vision.apriltag.*;
import com.frc6324.robot2026.subsystems.vision.objdetect.*;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@ExtensionMethod(PoseExtensions.class)
@SuppressWarnings("unused")
public class RobotContainer {
  private final AprilTagVision apriltag;
  private final Indexer indexer;
  private final Intake intake;
  private final LEDs leds = new LEDs();
  private final Rollers rollers;
  private final Shooter shooter;
  private final SwerveDrive drive;

  private final PowerDistribution pdh = new PowerDistribution();
  private final LoggedPowerDistribution loggedPDH =
      LoggedPowerDistribution.getInstance(pdh.getModule(), pdh.getType());
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Selection");

  private final CommandXboxController controller =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    pdh.setSwitchableChannel(true);

    switch (Constants.CURRENT_MODE) {
      case REAL -> {
        final DriveIOCTRE driveIO = new DriveIOCTRE();
        drive = new SwerveDrive(driveIO);

        intake = new Intake(new IntakeIOTalonFX());
        apriltag =
            new AprilTagVision(
                    new AprilTagIOPhoton(drive, intake::visionAvailable),
                    new AprilTagIOPhoton(drive),
                    new AprilTagIOPhoton(drive))
                .withConsumer(drive);
        indexer = new Indexer(new IndexerIOTalonFX());
        rollers = new Rollers(new RollerIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
      }
      case SIM -> {
        final DriveIOSim driveIO = new DriveIOSim();
        drive = new SwerveDrive(driveIO);

        apriltag =
            new AprilTagVision(
                new AprilTagIOSim(drive), new AprilTagIOSim(drive), new AprilTagIOSim(drive));
        indexer = new Indexer(new IndexerIOSim());
        intake = new Intake(new IntakeIOSim());
        rollers = new Rollers(new RollerIOSim());
        shooter = new Shooter(new ShooterIOSim());
      }
      default -> {
        drive = new SwerveDrive(new DriveIOReplay());

        apriltag = new AprilTagVision(IOLayer::replay, IOLayer::replay, IOLayer::replay);
        indexer = new Indexer(IOLayer::replay);
        intake = new Intake(IOLayer::replay);
        rollers = new Rollers(IOLayer::replay);
        shooter = new Shooter(IOLayer::replay);
      }
    }

    configureNamedCommands();
    configureBindings();

    autoChooser.addDefaultOption("No Auto", Commands.none());
    autoChooser.addOption(
        "Left Double Trench", AutoCommands.doubleTrenchAuto(AllianceSide.Left, intake));
    autoChooser.addOption(
        "Right Double Trench", AutoCommands.doubleTrenchAuto(AllianceSide.Right, intake));
    autoChooser.addOption(
        "Left Triple Trench", AutoCommands.tripleTrenchAuto(AllianceSide.Left, intake));
    autoChooser.addOption(
        "Right Triple Trench", AutoCommands.tripleTrenchAuto(AllianceSide.Right, intake));
    autoChooser.addOption(
        "Left Sweep-Pass Score", AutoCommands.sweepPassScoreAuto(AllianceSide.Left, intake));
    autoChooser.addOption(
        "Right Sweep-Pass Score", AutoCommands.sweepPassScoreAuto(AllianceSide.Right, intake));
    autoChooser.addOption("Depot-Outpost Score", AutoCommands.allianceZoneAuto());
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand(
        "Intake",
        Commands.run(
            () -> {
              intake.deploy(false);
              rollers.spinRollers();
            },
            intake,
            rollers));
    NamedCommands.registerCommand(
        "Outtake",
        Commands.run(
            () -> {
              intake.deploy(false);
              rollers.outtake();
            },
            intake,
            rollers));
    NamedCommands.registerCommand("RetractIntake", intake.run(intake::retract));
    NamedCommands.registerCommand("IdleShooter", new IdleShooterCommand(shooter, drive));
    NamedCommands.registerCommand(
        "ShootIntoHub",
        new ShootIntoHubCommand(drive, indexer, intake, rollers, shooter, "AutoShootIntoHub"));
  }

  private void configureBindings() {
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, controller.getHID()));
    shooter.setDefaultCommand(new IdleShooterCommand(shooter, drive));

    intake.setDefaultCommand(
        intake.run(
            () -> {
              if (drive
                  .getPose()
                  .boundedWithinX(
                      LinesVertical.NEUTRAL_ZONE_NEAR, LinesVertical.NEUTRAL_ZONE_FAR)) {
                intake.deploy(true);
              }
            }));
    rollers.setDefaultCommand(
        rollers.runEnd(
            () -> {
              final Pose2d drivePose = drive.getPose();
              if (drivePose.boundedWithinX(
                  LinesVertical.NEUTRAL_ZONE_NEAR, LinesVertical.NEUTRAL_ZONE_FAR)) {
                rollers.spinRollers();
                LEDState.intaking = true;
              } else {
                rollers.stopRollers();
                LEDState.intaking = false;
              }
            },
            () -> LEDState.intaking = false));

    indexer.setDefaultCommand(
        indexer.run(
            () -> {
              indexer.stopIndexerWheel();
              indexer.stopKickerWheel();
            }));

    controller
        .leftTrigger()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  intake.deploy(true);
                  rollers.spinRollers();

                  LEDState.intaking = true;
                },
                () -> LEDState.intaking = false,
                intake,
                rollers))
        .onFalse(rollers.run(rollers::stopRollers));

    controller.a().whileTrue(intake.run(intake::retract));
    controller.b().whileTrue(new ShootUpAgainstHubCommand(indexer, intake, rollers, shooter));

    controller
        .rightBumper()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  intake.deploy(true);
                  rollers.outtake();

                  LEDState.outtaking = true;
                },
                () -> LEDState.outtaking = false,
                rollers,
                intake));

    controller
        .rightTrigger()
        .whileTrue(
            ShooterCommands.genericShootCommand(
                drive, indexer, intake, rollers, shooter, controller));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
