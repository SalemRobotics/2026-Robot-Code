package com.frc6324.robot2026.subsystems.intake;

import static com.frc6324.robot2026.subsystems.intake.IntakeConstants.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
  private Distance extensionDistance = Inches.zero();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void deploy() {
    io.deploy();
  }

  public boolean isDeployed() {
    return inputs.motorPosition.isNear(INTAKE_DEPLOYED_POSITION, INTAKE_DEPLOY_TOLERANCE);
  }

  public boolean isStowed() {
    return inputs.motorPosition.isNear(INTAKE_STOWED_POSITION, INTAKE_DEPLOY_TOLERANCE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Deploy", inputs);

    final double factor =
        inputs.motorPosition.in(Rotations) / INTAKE_DEPLOYED_POSITION.in(Rotations);
    extensionDistance = INTAKE_EXTENSION.times(factor);

    Logger.recordOutput("Intake/DeployPosition", INTAKE_DEPLOYED_POSITION);
    Logger.recordOutput("Intake/MotorPositionRots", inputs.motorPosition.magnitude(), Rotations);

    Logger.recordOutput("Intake/ExtensionFactor", factor);
    Logger.recordOutput("Intake/ExtensionDistance", extensionDistance);
  }

  @Override
  public void simulationPeriodic() {
    // IntakeMechanism.getInstance().setExtension(extensionDistance);
    // IntakeMechanism.getInstance().log();
    Logger.recordOutput(
        "Intake/MechanismTransform",
        new Transform3d(extensionDistance, Inches.zero(), Inches.zero(), Rotation3d.kZero));
  }

  public void spring() {
    io.spring();
  }

  public void stow() {
    io.stow();
  }
}
