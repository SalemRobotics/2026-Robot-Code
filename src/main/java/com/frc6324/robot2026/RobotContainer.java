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
import com.frc6324.robot2026.subsystems.climber.Climber;
import com.frc6324.robot2026.subsystems.climber.ClimberIOSim;
import com.frc6324.robot2026.subsystems.climber.ClimberIOTalonFX;
import com.frc6324.robot2026.subsystems.drive.DriveIO.DriveIOReplay;
import com.frc6324.robot2026.subsystems.drive.DriveIOCTRE;
import com.frc6324.robot2026.subsystems.drive.DriveIOSim;
import com.frc6324.robot2026.subsystems.drive.SwerveDrive;
import com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagIOPhoton;
import com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagIOSim;
import com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagVision;
import com.frc6324.robot2026.subsystems.vision.objdetect.ObjectDetection;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Climber climber;
  private final SwerveDrive drive;

  @SuppressWarnings("unused")
  private final AprilTagVision visionOdometry;

  @SuppressWarnings("unused")
  private final ObjectDetection objectDetection;

  private final CommandXboxController controller =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL -> {
        climber = new Climber(new ClimberIOTalonFX());

        final DriveIOCTRE realDrive = new DriveIOCTRE();
        drive = new SwerveDrive(realDrive);

        visionOdometry =
            new AprilTagVision(
                    new AprilTagIOPhoton(realDrive::samplePoseAt),
                    new AprilTagIOPhoton(realDrive::samplePoseAt),
                    new AprilTagIOPhoton(realDrive::samplePoseAt),
                    new AprilTagIOPhoton(realDrive::samplePoseAt))
                .withConsumer(drive);

        objectDetection = new ObjectDetection();
      }
      case SIM -> {
        climber = new Climber(new ClimberIOSim());

        final DriveIOSim simDrive = new DriveIOSim();
        drive = new SwerveDrive(simDrive);

        visionOdometry =
            new AprilTagVision(
                new AprilTagIOSim(simDrive::samplePoseAt, drive),
                new AprilTagIOSim(simDrive::samplePoseAt, drive),
                new AprilTagIOSim(simDrive::samplePoseAt, drive),
                new AprilTagIOSim(simDrive::samplePoseAt, drive));
        objectDetection = new ObjectDetection();
      }
      default -> {
        climber = new Climber(IOLayer::replay);
        drive = new SwerveDrive(new DriveIOReplay());
        visionOdometry =
            new AprilTagVision(IOLayer::replay, IOLayer::replay, IOLayer::replay, IOLayer::replay);
        objectDetection = new ObjectDetection(IOLayer::replay);
      }
    }

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, controller.getHID()));
    // Bind climber commands to the D-pad

    controller.povUp().whileTrue(climber.stow()).onFalse(climber.stop());
    controller.povDown().whileTrue(climber.deploy()).onFalse(climber.stop());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
