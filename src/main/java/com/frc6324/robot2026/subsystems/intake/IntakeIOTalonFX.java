package com.frc6324.robot2026.subsystems.intake;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;
import static com.frc6324.robot2026.subsystems.intake.IntakeConstants.*;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  protected final TalonFX talon = new TalonFX(INTAKE_MOTOR_ID, INTAKE_CAN_BUS);
  private final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(0);
  // Status signals
  private final StatusSignal<Angle> deployPosition = talon.getPosition();
  private final StatusSignal<AngularVelocity> deployVelocity = talon.getVelocity();
  private final StatusSignal<Voltage> deployMotorVoltage = talon.getMotorVoltage();
  private final StatusSignal<Current> deployStatorCurrent = talon.getStatorCurrent();
  private final StatusSignal<Current> deployTorqueCurrent = talon.getTorqueCurrent();

  private final BaseStatusSignal[] signals = {
    deployPosition, deployVelocity, deployMotorVoltage, deployStatorCurrent, deployTorqueCurrent
  };

  public IntakeIOTalonFX() {
    // Set configurations for the motor
    tryUntilOk(5, () -> talon.getConfigurator().apply(INTAKE_MOTOR_CONFIG));
    tryUntilOk(5, () -> talon.setNeutralMode(NeutralModeValue.Coast));

    if (INTAKE_CAN_BUS == Constants.CANIVORE) {
      BaseStatusSignal.waitForAll(1, signals);
    }

    BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), signals);
    talon.optimizeBusUtilization(0);
  }

  @Override
  public void deploy() {
    talon.setControl(request.withPosition(INTAKE_DEPLOYED_POSITION).withSlot(INTAKE_DEPLOY_SLOT));
  }

  @Override
  public void spring() {
    talon.setControl(request.withPosition(INTAKE_DEPLOYED_POSITION).withSlot(INTAKE_SPRING_SLOT));
  }

  @Override
  public void stow() {
    talon.setControl(request.withPosition(INTAKE_STOWED_POSITION).withSlot(INTAKE_DEPLOY_SLOT));
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.refreshAll(signals).isOK();

    inputs.motorPosition = deployPosition.getValue();
    inputs.motorVelocity = deployVelocity.getValue();
    inputs.motorVoltage = deployMotorVoltage.getValue();
    inputs.motorStatorCurrent = deployStatorCurrent.getValue();
    inputs.motorTorqueCurrent = deployTorqueCurrent.getValue();
  }
}
