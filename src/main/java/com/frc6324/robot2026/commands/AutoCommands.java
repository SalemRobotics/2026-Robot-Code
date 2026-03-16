package com.frc6324.robot2026.commands;

import com.frc6324.lib.UninstantiableClass;
import com.frc6324.lib.util.PoseExtensions;
import com.frc6324.robot2026.commands.ShooterCommands.ShootIntoHubCommand;
import com.frc6324.robot2026.subsystems.drive.SwerveDrive;
import com.frc6324.robot2026.subsystems.indexer.Indexer;
import com.frc6324.robot2026.subsystems.intake.Intake;
import com.frc6324.robot2026.subsystems.rollers.Rollers;
import com.frc6324.robot2026.subsystems.shooter.Shooter;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(PoseExtensions.class)
@UninstantiableClass
public final class AutoCommands {
  public static enum AllianceSide {
    Left,
    Right;
  }

  private AutoCommands() {
    throw new IllegalAccessError();
  }

  public static Command trenchAuto(
      SwerveDrive drive,
      Indexer indexer,
      Intake intake,
      Rollers rollers,
      Shooter shooter,
      AllianceSide side) {
    PathPlannerPath intakePath1;

    try {
      intakePath1 = PathPlannerPath.fromPathFile("Trench First Intake");

    } catch (Exception e) {
      e.printStackTrace();

      return Commands.none().withName(side + " trench auto [MISSING PATH FILE]");
    }

    if (side == AllianceSide.Right) {
      intakePath1 = intakePath1.mirrorPath();
    }

    return Commands.sequence(
        Commands.parallel(intake.run(intake::deploy), shooter.run(shooter::stowHood))
            .until(intake::isDeployed),
        Commands.waitSeconds(0.2),
        AutoBuilder.followPath(intakePath1)
            .alongWith(intake.runOnce(intake::deploy), rollers.runOnce(rollers::spinRollers)),
        new ShootIntoHubCommand(drive, indexer, intake, rollers, shooter, "AutoShootIntoHub"));
  }
}
