package com.frc6324.robot2026.subsystems.intake;

import static com.frc6324.robot2026.subsystems.intake.IntakeConstants.*;
import static edu.wpi.first.units.Units.*;

import com.frc6324.robot2026.sim.MapleSimManager;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public final class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
  private Distance extensionDistance = Inches.zero();

  public final Trigger isDeployed = new Trigger(this::isDeployed);
  public final Trigger isStowed = new Trigger(this::isStowed);

  public Intake(IntakeIO io) {
    this.io = io;

    if (RobotBase.isSimulation()) {
      isDeployed.onTrue(
          Commands.runOnce(() -> MapleSimManager.getInstance().setIntakeExtended(true)));
      isDeployed.onFalse(
          Commands.runOnce(() -> MapleSimManager.getInstance().setIntakeExtended(false)));
    }
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
  }

  @Override
  public void simulationPeriodic() {
    Translation3d mechTranslation =
        new Translation3d(extensionDistance, Inches.zero(), Inches.zero())
            .rotateBy(INTAKE_MECHANISM_ROTATION);

    Logger.recordOutput(
        "Intake/MechanismTransform", new Transform3d(mechTranslation, Rotation3d.kZero));
  }

  public void spring() {
    io.spring();
  }

  public void stow() {
    io.stow();
  }
}
