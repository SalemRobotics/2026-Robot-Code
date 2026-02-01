package com.frc6324.robot2026.subsystems.intake;

import static com.frc6324.robot2026.subsystems.intake.IntakeConstants.*;
import static edu.wpi.first.units.Units.Inches;

import com.frc6324.robot2026.mechanisms.IntakeMechanism;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
  private Distance extensionDistance = Inches.zero();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public Command deploy() {
    return Commands.sequence(
        runOnce(io::deploy),
        idle()
            .until(
                () ->
                    inputs.motorPosition.isNear(INTAKE_DEPLOYED_POSITION, INTAKE_DEPLOY_TOLERANCE)),
        runOnce(io::spring),
        idle());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Deploy", inputs);

    extensionDistance =
        INTAKE_EXTENSION.times(INTAKE_DEPLOYED_POSITION.div(inputs.motorPosition).magnitude());
  }

  @Override
  public void simulationPeriodic() {
    IntakeMechanism.getInstance().setExtension(extensionDistance);
  }

  public Command stow() {
    return runOnce(io::stow).andThen(idle());
  }
}
