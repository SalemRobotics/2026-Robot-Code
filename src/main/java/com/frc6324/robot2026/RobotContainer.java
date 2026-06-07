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
import com.frc6324.lib.util.LoggedTracer;
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
        LoggedTracer.reset();
        final DriveIOCTRE driveIO = new DriveIOCTRE();
        drive = new SwerveDrive(driveIO);
        LoggedTracer.record("Init/Drive init");

        intake = new Intake(new IntakeIOTalonFX());
        LoggedTracer.record("Init/Intake init");
        apriltag =
            new AprilTagVision(
                    new AprilTagIOPhoton(driveIO),
                    new AprilTagIOPhoton(driveIO),
                    new AprilTagIOPhoton(driveIO))
                .withConsumer(drive);
        LoggedTracer.record("Init/AprilTag init");

        indexer = new Indexer(new IndexerIOTalonFX());
        LoggedTracer.record("Init/Indexer init");

        rollers = new Rollers(new RollerIOTalonFX());
        LoggedTracer.record("Init/Rollers init");

        shooter = new Shooter(new ShooterIOTalonFX());
        LoggedTracer.record("Init/Shooter init");
      }
      case SIM -> {
        LoggedTracer.reset();
        final DriveIOSim driveIO = new DriveIOSim();
        drive = new SwerveDrive(driveIO);
        LoggedTracer.record("Init/Drive init");

        apriltag =
            new AprilTagVision(
                new AprilTagIOSim(driveIO, drive),
                new AprilTagIOSim(driveIO, drive),
                new AprilTagIOSim(driveIO, drive));
        LoggedTracer.record("Init/AprilTag init");

        indexer = new Indexer(new IndexerIOSim());
        LoggedTracer.record("Init/Indexer init");

        intake = new Intake(new IntakeIOSim());
        LoggedTracer.record("Init/Intake init");

        rollers = new Rollers(new RollerIOSim());
        LoggedTracer.record("Init/Rollers init");

        shooter = new Shooter(new ShooterIOSim());
        LoggedTracer.record("Init/Shooter init");
      }
      default -> {
        LoggedTracer.reset();
        drive = new SwerveDrive(new DriveIOReplay());
        LoggedTracer.record("Init/Drive init");

        apriltag = new AprilTagVision(IOLayer::replay, IOLayer::replay, IOLayer::replay);
        LoggedTracer.record("Init/AprilTag init");

        indexer = new Indexer(IOLayer::replay);
        LoggedTracer.record("Init/Indexer init");

        intake = new Intake(IOLayer::replay);
        LoggedTracer.record("Init/Intake init");

        rollers = new Rollers(IOLayer::replay);
        LoggedTracer.record("Init/Rollers init");

        shooter = new Shooter(IOLayer::replay);
        LoggedTracer.record("Init/Shooter init");
      }
    }

    configureNamedCommands();
    LoggedTracer.record("Init/Auto command bindings");

    configureBindings();
    LoggedTracer.record("Init/Controller Bindings");

    autoChooser.addDefaultOption("No Auto", Commands.none());
    autoChooser.addOption(
        "Left Double Trench", AutoCommands.doubleTrenchAuto(AllianceSide.Left, intake));
    autoChooser.addOption(
        "Right Double Trench", AutoCommands.doubleTrenchAuto(AllianceSide.Right, intake));

    autoChooser.addOption(
        "Nashoba Left Double Trench", AutoCommands.nashobaDoubleTrenchAuto(AllianceSide.Left));
    autoChooser.addOption(
        "Nashoba Right Double Trench", AutoCommands.nashobaDoubleTrenchAuto(AllianceSide.Right));

    autoChooser.addOption(
        "Force Left Double Trench", AutoCommands.forceDoubleTrenchAuto(AllianceSide.Left));
    autoChooser.addOption(
        "Force Right Double Trench", AutoCommands.forceDoubleTrenchAuto(AllianceSide.Right));

    autoChooser.addOption(
        "Left Double Trench Reversed",
        AutoCommands.doubleTrenchReversedAuto(AllianceSide.Left, intake));

    autoChooser.addOption(
        "Right Double Trench Reversed",
        AutoCommands.doubleTrenchReversedAuto(AllianceSide.Right, intake));

    LoggedTracer.record("Init/Auto chooser");
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
            () -> {
              rollers.stopRollers();
              LEDState.intaking = false;
            }));

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

    controller.povUp().onTrue(Commands.runOnce(shooter::incrementOffset));
    controller.povDown().onTrue(Commands.runOnce(shooter::decrementOffset));
    controller.back().onTrue(Commands.runOnce(shooter::resetOffset));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
