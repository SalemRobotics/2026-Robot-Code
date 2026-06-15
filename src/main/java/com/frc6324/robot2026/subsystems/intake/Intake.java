package com.frc6324.robot2026.subsystems.intake;

import static com.frc6324.robot2026.subsystems.intake.IntakeConstants.*;
import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.util.CommonUtils;
import com.frc6324.lib.util.logging.LoggedTracer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod(CommonUtils.class)
public final class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
  private Distance extensionDistance = Inches.zero();

  public final Trigger isDeployed = new Trigger(this::isDeployed);
  public final Trigger isStowed = new Trigger(this::isStowed);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void deploy(boolean useEfficiencyMode) {
    final int slot = useEfficiencyMode ? INTAKE_EFFICIENCY_SLOT : INTAKE_FAST_SLOT;
    io.setPosition(INTAKE_DEPLOYED_POSITION, slot);
  }

  public boolean isDeployed() {
    return inputs.motorPosition.isNear(INTAKE_DEPLOYED_POSITION, INTAKE_DEPLOY_TOLERANCE);
  }

  public boolean isRetracted() {
    return inputs.motorPosition.isNear(INTAKE_RETRACTED_POSITION, INTAKE_DEPLOY_TOLERANCE);
  }

  public boolean isSafeToTrench() {
    return inputs.motorPosition.gte(INTAKE_STOWED_POSITION);
  }

  public boolean isStowed() {
    return inputs.motorPosition.isNear(INTAKE_STOWED_POSITION, INTAKE_DEPLOY_TOLERANCE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Deploy", inputs);

    // Unary minus is there bc the motor is inverted IRL
    extensionDistance =
        INTAKE_EXTENSION.times(inputs.motorPosition.div(INTAKE_DEPLOYED_POSITION).magnitude());

    LoggedTracer.record("Periodic/Intake periodic");
  }

  public void retract() {
    io.setPositionProfiled(INTAKE_RETRACTED_POSITION, INTAKE_RETRACT_SLOT);
  }

  @Override
  public void simulationPeriodic() {
    Translation3d mechTranslation =
        new Translation3d(extensionDistance, Inches.zero(), Inches.zero())
            .rotateBy(INTAKE_MECHANISM_ROTATION);

    Logger.recordOutput(
        "Intake/MechanismTransform", new Transform3d(mechTranslation, Rotation3d.kZero));
  }

  public boolean shouldSendOut() {
    return inputs.motorPosition.gt(INTAKE_DEPLOYED_POSITION.plus(INTAKE_RETRACTED_POSITION).div(2));
  }

  public void spring() {
    io.spring();
  }

  public void stow(boolean useEfficiencyMode) {
    final int slot = useEfficiencyMode ? INTAKE_EFFICIENCY_SLOT : INTAKE_FAST_SLOT;
    io.setPosition(INTAKE_STOWED_POSITION, slot);
  }

  public boolean visionAvailable() {
    return inputs.motorPosition.gte(INTAKE_VISION_THRESHOLD);
  }
}
