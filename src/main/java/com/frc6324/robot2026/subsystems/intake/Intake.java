package com.frc6324.robot2026.subsystems.intake;

import static com.frc6324.robot2026.subsystems.intake.IntakeConstants.*;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;

import com.frc6324.robot2026.mechanisms.IntakeMechanism;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
  private Distance extensionDistance = Inches.zero();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public Command deploy() {
    return Commands.parallel(Commands.sequence(runOnce(io::deploy), idle().until(() -> inputs.deployPosition.isNear(INTAKE_DEPLOYED_POSITION, INTAKE_DEPLOY_TOLERANCE)), runOnce(io::spring), idle()).finallyDo(io::stopDeploy));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    extensionDistance = INTAKE_EXTENSION.times(INTAKE_DEPLOYED_POSITION.div(inputs.deployPosition).magnitude());
  }

  @Override
  public void simulationPeriodic() {
    IntakeMechanism.getInstance().setExtension(extensionDistance);
  }

  public Command startRollers() {
    return runOnce(io::runRollers);
  }

  public Command stopRollers() {
    return runOnce(io::stopRollers);
  }

  public Command stow() {
    return startEnd(io::stow, io::stopDeploy)
      .until(() -> inputs.deployPosition.isNear(INTAKE_STOWED_POSITION, INTAKE_DEPLOY_TOLERANCE));
  }
}
