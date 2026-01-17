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

import com.frc6324.lib.util.IOLayer;
import com.frc6324.lib.util.PoseExtensions;
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
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(PoseExtensions.class)
public class RobotContainer {
  private final SwerveDrive drive;
  private final AprilTagVision visionOdometry;
  private final ObjectDetection objectDetection;

  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL -> {
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
        drive = new SwerveDrive(new DriveIOReplay());
        visionOdometry =
            new AprilTagVision(IOLayer::replay, IOLayer::replay, IOLayer::replay, IOLayer::replay);
        objectDetection = new ObjectDetection(IOLayer::replay);
      }
    }

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
