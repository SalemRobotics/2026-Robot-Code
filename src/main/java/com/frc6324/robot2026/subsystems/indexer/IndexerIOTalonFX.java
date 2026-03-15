package com.frc6324.robot2026.subsystems.indexer;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;
import static com.frc6324.robot2026.subsystems.indexer.IndexerConstants.*;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.frc6324.lib.util.PhoenixUtil;
import edu.wpi.first.units.measure.*;

public class IndexerIOTalonFX implements IndexerIO {
  protected final TalonFX spinner = new TalonFX(INDEXER_SPINNER_MOTOR_ID, INDEXER_CAN_BUS);
  protected final TalonFX kicker = new TalonFX(INDEXER_FEEDER_MOTOR_ID, INDEXER_CAN_BUS);

  private final VelocityTorqueCurrentFOC kickerRequest =
      new VelocityTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);
  private final VelocityTorqueCurrentFOC spinnerRequest =
      new VelocityTorqueCurrentFOC(0)
          .withOverrideCoastDurNeutral(true)
          .withUpdateFreqHz(Hertz.of(500));

  private final StatusSignal<AngularVelocity> kickerVelocity = kicker.getVelocity();
  private final StatusSignal<Voltage> kickerVoltage = kicker.getMotorVoltage();
  private final StatusSignal<Current> kickerStatorCurrent = kicker.getStatorCurrent();
  private final BaseStatusSignal[] kickerSignals = {
    kickerVelocity, kickerVoltage, kickerStatorCurrent,
  };

  private final StatusSignal<AngularVelocity> spinnerVelocity = spinner.getVelocity();
  private final StatusSignal<Voltage> spinnerVoltage = spinner.getMotorVoltage();
  private final StatusSignal<Current> spinnerStatorCurrent = spinner.getStatorCurrent();
  private final BaseStatusSignal[] spinnerSignals = {
    spinnerVelocity, spinnerVoltage, spinnerStatorCurrent,
  };

  public IndexerIOTalonFX() {
    PhoenixUtil.addSignals(kicker, kickerSignals);
    PhoenixUtil.addSignals(spinner, spinnerSignals);
    ParentDevice.optimizeBusUtilizationForAll(0, kicker, spinner);

    spinner.setSafetyEnabled(false);

    tryUntilOk(5, () -> kicker.getConfigurator().apply(INDEXER_KICKER_CONFIG));
    tryUntilOk(5, () -> kicker.setNeutralMode(NeutralModeValue.Brake));
    tryUntilOk(5, () -> spinner.getConfigurator().apply(INDEXER_SPINNER_CONFIG));
    tryUntilOk(5, () -> spinner.setNeutralMode(NeutralModeValue.Brake));
  }

  @Override
  public void setKickerVelocity(AngularVelocity velocity) {
    kicker.setControl(kickerRequest.withVelocity(velocity));
  }

  @Override
  public void setSpinnerVelocity(AngularVelocity velocity) {
    spinner.setControl(spinnerRequest.withVelocity(velocity));
  }

  @Override
  public void stopKicker() {
    kicker.stopMotor();
  }

  @Override
  public void stopSpinner() {
    spinner.stopMotor();
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.kickerMotorConnected = BaseStatusSignal.isAllGood(kickerSignals);
    inputs.kickerVelocity = kickerVelocity.getValue();
    inputs.kickerMotorVoltage = kickerVoltage.getValue();
    inputs.kickerStatorCurrent = kickerStatorCurrent.getValue();

    inputs.spinnerMotorConnected = BaseStatusSignal.isAllGood(spinnerSignals);
    inputs.spinnerVelocity = spinnerVelocity.getValue();
    inputs.spinnerMotorVoltage = spinnerVoltage.getValue();
    inputs.spinnerStatorCurrent = spinnerStatorCurrent.getValue();
  }
}
