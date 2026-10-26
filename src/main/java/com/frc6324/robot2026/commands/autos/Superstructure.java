package com.frc6324.robot2026.commands.autos;

import com.frc6324.lib.auto.AutoBuilder.SuperstructureBase;
import com.frc6324.robot2026.Constants;
import com.frc6324.robot2026.Constants.Mode;
import com.frc6324.robot2026.RobotState;
import com.frc6324.robot2026.commands.ShooterCommands.ShootIntoHubCommand;
import com.frc6324.robot2026.subsystems.drive.Drive;
import com.frc6324.robot2026.subsystems.indexer.Indexer;
import com.frc6324.robot2026.subsystems.intake.Intake;
import com.frc6324.robot2026.subsystems.rollers.Rollers;
import com.frc6324.robot2026.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Map;

public class Superstructure implements SuperstructureBase {
  private final RobotState robotState;
  private final Drive drive;
  private final Indexer indexer;
  private final Intake intake;
  private final Rollers rollers;
  private final Shooter shooter;

  public Superstructure(
      RobotState robotState,
      Drive drive,
      Indexer indexer,
      Intake intake,
      Rollers rollers,
      Shooter shooter) {
    this.robotState = robotState;
    this.drive = drive;
    this.indexer = indexer;
    this.intake = intake;
    this.rollers = rollers;
    this.shooter = shooter;
  }

  @Override
  public void registerEventBindings(Map<String, Command> bindings) {}

  /**
   * Blocks auto until the intake is safe to go under the trench.
   *
   * @return
   */
  public Command initialIntakeDeploy() {
    final Command deadline =
        Constants.CURRENT_MODE == Mode.REAL
            ? Commands.waitUntil(intake::isSafeToTrench)
            : Commands.waitSeconds(0.4);

    return Commands.deadline(deadline, intake.run(() -> intake.deploy(false)));
  }

  public Command passiveIntake() {
    return Commands.runEnd(
        () -> {
          intake.deploy(true);
          rollers.spinRollers();
        },
        () -> rollers.stopRollers(),
        intake,
        rollers);
  }

  public Command shootCommand() {
    return new ShootIntoHubCommand(
        robotState, drive, indexer, intake, rollers, shooter, "AutoShootIntoHub");
  }

  public Command shootCommand(double timeout) {
    return shootCommand().withTimeout(timeout);
  }
}
