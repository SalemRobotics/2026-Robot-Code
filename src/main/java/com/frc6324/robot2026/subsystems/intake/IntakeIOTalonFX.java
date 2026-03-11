package com.frc6324.robot2026.subsystems.intake;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;
import static com.frc6324.robot2026.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.frc6324.lib.util.PhoenixUtil;
import edu.wpi.first.units.measure.*;

public class IntakeIOTalonFX implements IntakeIO {
  protected final TalonFX talon = new TalonFX(INTAKE_MOTOR_ID, INTAKE_CAN_BUS);
  private final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(0);
  // Status signals
  private final StatusSignal<Angle> deployPosition = talon.getPosition();
  private final StatusSignal<Double> pidSetpoint = talon.getClosedLoopReference();
  private final StatusSignal<Double> positionError = talon.getClosedLoopError();
  private final StatusSignal<Double> pidOutput = talon.getClosedLoopOutput();
  private final StatusSignal<AngularVelocity> deployVelocity = talon.getVelocity();
  private final StatusSignal<Voltage> deployMotorVoltage = talon.getMotorVoltage();
  private final StatusSignal<Current> deployStatorCurrent = talon.getStatorCurrent();
  private final StatusSignal<Current> deployTorqueCurrent = talon.getTorqueCurrent();

  private final BaseStatusSignal[] signals = {
    deployPosition,
    positionError,
    pidOutput,
    deployVelocity,
    deployMotorVoltage,
    deployStatorCurrent,
    deployTorqueCurrent
  };

  public IntakeIOTalonFX() {
    PhoenixUtil.addSignals(talon, signals);
    talon.optimizeBusUtilization(0);

    // Set configurations for the motor
    tryUntilOk(5, () -> talon.getConfigurator().apply(INTAKE_MOTOR_CONFIG));
    tryUntilOk(5, () -> talon.setNeutralMode(NeutralModeValue.Coast));
  }

  @Override
  public void setPosition(Angle position) {
    talon.setControl(request.withPosition(position).withSlot(INTAKE_MOVE_SLOT));
  }

  @Override
  public void spring() {
    talon.setControl(request.withPosition(INTAKE_DEPLOYED_POSITION).withSlot(INTAKE_SPRING_SLOT));
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.isAllGood(signals);

    inputs.motorPosition = deployPosition.getValue();
    inputs.positionError = positionError.getValueAsDouble();
    inputs.pidSetpoint = pidSetpoint.getValueAsDouble();
    inputs.pidOutput = pidOutput.getValueAsDouble();
    inputs.motorVelocity = deployVelocity.getValue();
    inputs.motorVoltage = deployMotorVoltage.getValue();
    inputs.motorStatorCurrent = deployStatorCurrent.getValue();
    inputs.motorTorqueCurrent = deployTorqueCurrent.getValue();
  }
}
