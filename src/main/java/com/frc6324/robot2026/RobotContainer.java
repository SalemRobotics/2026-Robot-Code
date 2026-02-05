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

import com.frc6324.lib.util.IOLayer;
import com.frc6324.robot2026.commands.DriveCommands;
import com.frc6324.robot2026.subsystems.climber.*;
import com.frc6324.robot2026.subsystems.drive.*;
import com.frc6324.robot2026.subsystems.drive.DriveIO.DriveIOReplay;
import com.frc6324.robot2026.subsystems.intake.*;
import com.frc6324.robot2026.subsystems.leds.LEDs;
import com.frc6324.robot2026.subsystems.rollers.*;
import com.frc6324.robot2026.subsystems.vision.apriltag.*;
import com.frc6324.robot2026.subsystems.vision.objdetect.*;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.LoggedPowerDistribution;

@SuppressWarnings("unused")
public class RobotContainer {
  private final AprilTagVision visionOdometry;
  private final Climber climber;
  private final Intake intake;
  private final LEDs leds = new LEDs();
  private final ObjectDetection objectDetection;
  private final Rollers rollers;
  private final SwerveDrive drive;

  private final LoggedPowerDistribution pdh =
      LoggedPowerDistribution.getInstance(0, ModuleType.kRev);
  private final CommandXboxController controller =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL -> {
        final DriveIOCTRE driveIO = new DriveIOCTRE();
        drive = new SwerveDrive(driveIO);

        visionOdometry =
            new AprilTagVision(
                    new AprilTagIOPhoton(driveIO),
                    new AprilTagIOPhoton(driveIO),
                    new AprilTagIOPhoton(driveIO),
                    new AprilTagIOPhoton(driveIO))
                .withConsumer(drive);
        objectDetection = new ObjectDetection(new ObjDetectIOPhoton());

        intake = new Intake(new IntakeIOTalonFX());
        rollers = new Rollers(new RollerIOTalonFX());

        climber = new Climber(new ClimberIOTalonFX());
      }
      case SIM -> {
        final DriveIOSim driveIO = new DriveIOSim();
        drive = new SwerveDrive(driveIO);

        visionOdometry =
            new AprilTagVision(
                new AprilTagIOSim(driveIO, drive),
                new AprilTagIOSim(driveIO, drive),
                new AprilTagIOSim(driveIO, drive),
                new AprilTagIOSim(driveIO, drive));
        objectDetection = new ObjectDetection(IOLayer::replay);

        intake = new Intake(new IntakeIOSim());
        rollers = new Rollers(new RollerIOSim());

        climber = new Climber(new ClimberIOSim());
      }
      default -> {
        drive = new SwerveDrive(new DriveIOReplay());
        visionOdometry =
            new AprilTagVision(IOLayer::replay, IOLayer::replay, IOLayer::replay, IOLayer::replay);
        objectDetection = new ObjectDetection(IOLayer::replay);
        intake = new Intake(IOLayer::replay);
        rollers = new Rollers(IOLayer::replay);
        climber = new Climber(IOLayer::replay);
      }
    }

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, controller.getHID()));
    // Bind climber commands to the D-pad

    controller.povUp().whileTrue(climber.stow()).onFalse(climber.stop());
    controller.povDown().whileTrue(climber.deploy()).onFalse(climber.stop());

    controller
        .y()
        .whileTrue(
            Commands.print("Deploying intake!")
                .andThen(Commands.run(intake::deploy, intake).until(intake::isDeployed)))
        .onFalse(
            Commands.print("Stowing intake!")
                .andThen(Commands.run(intake::stow, intake).until(intake::isStowed)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
