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
import com.frc6324.robot2026.subsystems.rollers.*;
import com.frc6324.robot2026.subsystems.shooter.Shooter;
import com.frc6324.robot2026.subsystems.shooter.ShooterIOSim;
import com.frc6324.robot2026.subsystems.shooter.ShooterIOTalonFX;
import com.frc6324.robot2026.subsystems.vision.apriltag.*;
import com.frc6324.robot2026.subsystems.vision.objdetect.*;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.LoggedPowerDistribution;

@SuppressWarnings("unused")
public class RobotContainer {
  private final Intake intake;
  private final Shooter shooter;
  private final SwerveDrive drive;

  private final PowerDistribution pdh = new PowerDistribution();
  private final LoggedPowerDistribution loggedPDH =
      LoggedPowerDistribution.getInstance(pdh.getModule(), pdh.getType());
  private final CommandXboxController controller =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    pdh.setSwitchableChannel(true);

    switch (Constants.CURRENT_MODE) {
      case REAL -> {
        final DriveIOCTRE driveIO = new DriveIOCTRE();
        drive = new SwerveDrive(driveIO);

        intake = new Intake(new IntakeIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
      }
      case SIM -> {
        final DriveIOSim driveIO = new DriveIOSim();
        drive = new SwerveDrive(driveIO);

        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
      }
      default -> {
        drive = new SwerveDrive(new DriveIOReplay());

        intake = new Intake(IOLayer::replay);
        shooter = new Shooter(IOLayer::replay);
      }
    }

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, controller.getHID()));
    shooter.setDefaultCommand(
        Commands.runOnce(shooter::spinUpShooter, shooter).andThen(shooter.idle()));

    controller
        .x()
        .whileTrue(Commands.run(shooter::pass, shooter))
        .onFalse(
            Commands.run(
                () -> {
                  shooter.spinUpShooter();
                  shooter.stowHood();
                },
                shooter));

    controller
        .y()
        .whileTrue(Commands.run(intake::deploy, intake).until(intake::isDeployed))
        .onFalse(Commands.run(intake::stow, intake).until(intake::isStowed));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
