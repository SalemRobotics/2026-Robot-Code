package com.frc6324.robot2026.commands;

import com.frc6324.lib.UninstantiableClass;
import com.frc6324.lib.util.PoseExtensions;
import com.frc6324.robot2026.commands.ShooterCommands.IdleShooterCommand;
import com.frc6324.robot2026.commands.ShooterCommands.ShootIntoHubCommand;
import com.frc6324.robot2026.subsystems.drive.SwerveDrive;
import com.frc6324.robot2026.subsystems.indexer.Indexer;
import com.frc6324.robot2026.subsystems.intake.Intake;
import com.frc6324.robot2026.subsystems.rollers.Rollers;
import com.frc6324.robot2026.subsystems.shooter.Shooter;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.RobotBase;
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

  private static Command followAutoPath(
      PathPlannerPath pathToFollow,
      SwerveDrive drive,
      Intake intake,
      Rollers rollers,
      Shooter shooter) {
    return AutoBuilder.followPath(pathToFollow)
        .alongWith(
            new IdleShooterCommand(shooter, drive),
            intake.runOnce(() -> intake.deploy(true)),
            rollers.runOnce(rollers::spinRollers));
  }

  public static Command trenchDepotTrenchAuto(
      SwerveDrive drive, Indexer indexer, Intake intake, Rollers rollers, Shooter shooter) {
    final PathPlannerPath firstNZIntake, depotIntake, centerToTrench, secondNZIntake;

    try {
      firstNZIntake = PathPlannerPath.fromPathFile("Trench Start Intake");
      depotIntake = PathPlannerPath.fromPathFile("Trench to Depot Intake");
      centerToTrench = PathPlannerPath.fromPathFile("Center Score to Trench");
      secondNZIntake = PathPlannerPath.fromPathFile("Trench Score Intake");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return Commands.sequence(
        Commands.parallel(
                intake.run(() -> intake.deploy(false)),
                rollers.run(rollers::spinRollers),
                shooter.run(shooter::stowHood))
            .until(intake::isSafeToTrench),
        followAutoPath(firstNZIntake, drive, intake, rollers, shooter),
        new ShootIntoHubCommand(drive, indexer, intake, rollers, shooter, "AutoShootIntoHub")
            .withTimeout(5),
        followAutoPath(depotIntake, drive, intake, rollers, shooter),
        new ShootIntoHubCommand(drive, indexer, intake, rollers, shooter, "AutoShootIntoHub")
            .withTimeout(5),
        followAutoPath(centerToTrench, drive, intake, rollers, shooter),
        followAutoPath(secondNZIntake, drive, intake, rollers, shooter),
        new ShootIntoHubCommand(drive, indexer, intake, rollers, shooter, "AutoShootIntoHub"));
  }

  public static Command tripleTrenchAuto(AllianceSide side, Intake intake) {
    return Commands.deadline(
            Commands.waitUntil(intake::isSafeToTrench), intake.run(() -> intake.deploy(false)))
        .onlyIf(RobotBase::isReal)
        .andThen(new PathPlannerAuto("Triple Trench", side == AllianceSide.Right));
  }

  public static Command sweepPassScoreAuto(AllianceSide side, Intake intake) {
    return Commands.deadline(
            Commands.waitUntil(intake::isSafeToTrench), intake.run(() -> intake.deploy(false)))
        .onlyIf(RobotBase::isReal)
        .andThen(new PathPlannerAuto("Sweep Pass Shoot", side == AllianceSide.Right));
  }
}
