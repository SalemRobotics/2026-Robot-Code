package com.frc6324.robot2026.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  public Command deploy() {
    return run(io::deploy);
  }

  @Override
  public void periodic() {
    // Only logs climber inputs since it doesn't do anything else
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command stop() {
    return runOnce(io::stop);
  }

  public Command stow() {
    return runOnce(io::stow);
  }
}
