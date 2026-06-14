package com.frc6324.robot2026.subsystems.indexer;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;
import static com.frc6324.robot2026.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.frc6324.lib.util.PhoenixUtil.SignalCache;
import edu.wpi.first.units.measure.*;

public class IndexerIOTalonFX implements IndexerIO {
  protected final TalonFX belt = new TalonFX(INDEXER_BELT_MOTOR_ID, INDEXER_CAN_BUS);
  protected final TalonFX kicker = new TalonFX(INDEXER_FEEDER_MOTOR_ID, INDEXER_CAN_BUS);

  private final VelocityTorqueCurrentFOC kickerRequest =
      new VelocityTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);

  private final StatusSignal<AngularVelocity> kickerVelocity = kicker.getVelocity();
  private final StatusSignal<Voltage> kickerVoltage = kicker.getMotorVoltage();
  private final StatusSignal<Current> kickerStatorCurrent = kicker.getStatorCurrent();
  private final BaseStatusSignal[] kickerSignals = {
    kickerVelocity, kickerVoltage, kickerStatorCurrent,
  };

  private final StatusSignal<AngularVelocity> beltVelocity = belt.getVelocity();
  private final StatusSignal<Voltage> beltVoltage = belt.getMotorVoltage();
  private final StatusSignal<Current> beltStatorCurrent = belt.getStatorCurrent();
  private final BaseStatusSignal[] beltSignals = {
    beltVelocity, beltVoltage, beltStatorCurrent,
  };

  public IndexerIOTalonFX() {
    SignalCache.addSignals(kicker, kickerSignals);
    SignalCache.addSignals(belt, beltSignals);
    ParentDevice.optimizeBusUtilizationForAll(0, kicker, belt);

    tryUntilOk(5, () -> kicker.getConfigurator().apply(INDEXER_KICKER_CONFIG));
    tryUntilOk(5, () -> kicker.setNeutralMode(NeutralModeValue.Brake));
    tryUntilOk(5, () -> belt.getConfigurator().apply(INDEXER_BELT_CONFIG));
    tryUntilOk(5, () -> belt.setNeutralMode(NeutralModeValue.Coast));
  }

  @Override
  public void setKickerVelocity(AngularVelocity velocity) {
    kicker.setControl(kickerRequest.withVelocity(velocity));
  }

  @Override
  public void setBeltVelocity(AngularVelocity velocity) {
    belt.set(1);
  }

  @Override
  public void stopKicker() {
    kicker.stopMotor();
  }

  @Override
  public void stopBelt() {
    belt.stopMotor();
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.kickerMotorConnected = BaseStatusSignal.isAllGood(kickerSignals);
    inputs.kickerVelocity = kickerVelocity.getValue();
    inputs.kickerMotorVoltage = kickerVoltage.getValue();
    inputs.kickerStatorCurrent = kickerStatorCurrent.getValue();

    inputs.spinnerMotorConnected = BaseStatusSignal.isAllGood(beltSignals);
    inputs.spinnerVelocity = beltVelocity.getValue();
    inputs.spinnerMotorVoltage = beltVoltage.getValue();
    inputs.spinnerStatorCurrent = beltStatorCurrent.getValue();
  }
}
