package com.frc6324.robot2026.commands;

import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.subsystems.intake.Intake;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@UninstantiableClass
public final class AutoCommands {
  public static enum AllianceSide {
    Left,
    Right;
  }

  private AutoCommands() {
    throw new IllegalAccessError();
  }

  public static Command doubleTrenchAuto(AllianceSide side, Intake intake) {
    return Commands.deadline(
            Commands.waitUntil(intake::isSafeToTrench), intake.run(() -> intake.deploy(false)))
        .onlyIf(RobotBase::isReal)
        .andThen(new PathPlannerAuto("Double Trench", side == AllianceSide.Right));
  }

  public static Command doubleTrenchReversedAuto(AllianceSide side, Intake intake) {
    return Commands.deadline(
            Commands.waitUntil(intake::isSafeToTrench), intake.run(() -> intake.deploy(false)))
        .onlyIf(RobotBase::isReal)
        .andThen(new PathPlannerAuto("Double Trench Reversed", side == AllianceSide.Right));
  }
}
