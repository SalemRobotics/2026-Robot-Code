package com.frc6324.robot2026.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Rollers extends SubsystemBase {
  private final RollerIO io;
  private final RollerInputsAutoLogged inputs = new RollerInputsAutoLogged();

  public Rollers(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Rollers", inputs);
  }

  public Command startRollers() {
    return runOnce(io::start);
  }

  public Command stopRollers() {
    return runOnce(io::stop);
  }
}
