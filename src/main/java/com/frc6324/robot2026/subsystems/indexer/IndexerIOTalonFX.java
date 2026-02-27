package com.frc6324.robot2026.subsystems.indexer;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;
import static com.frc6324.robot2026.subsystems.indexer.IndexerConstants.*;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.units.measure.*;

public class IndexerIOTalonFX implements IndexerIO {
  protected final TalonFX spinner = new TalonFX(INDEXER_SPINNER_MOTOR_ID, INDEXER_CAN_BUS);
  protected final TalonFX kicker = new TalonFX(INDEXER_FEEDER_MOTOR_ID, INDEXER_CAN_BUS);

  private final VelocityTorqueCurrentFOC kickerRequest =
      new VelocityTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);
  private final VelocityTorqueCurrentFOC spinnerRequest =
      new VelocityTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);

  private final StatusSignal<AngularVelocity> kickerVelocity = kicker.getVelocity();
  private final StatusSignal<AngularAcceleration> kickerAcceleration = kicker.getAcceleration();
  private final StatusSignal<Voltage> kickerVoltage = kicker.getMotorVoltage();
  private final StatusSignal<Current> kickerStatorCurrent = kicker.getStatorCurrent();
  private final StatusSignal<Current> kickerTorqueCurrent = kicker.getTorqueCurrent();
  private final StatusSignal<Double> kickerTargetVelocity = kicker.getClosedLoopReference();
  private final StatusSignal<Double> kickerPIDOutput = kicker.getClosedLoopOutput();
  private final BaseStatusSignal[] kickerSignals = {
    kickerVelocity,
    kickerAcceleration,
    kickerVoltage,
    kickerStatorCurrent,
    kickerTorqueCurrent,
    kickerTargetVelocity,
    kickerPIDOutput,
  };

  private final StatusSignal<AngularVelocity> spinnerVelocity = spinner.getVelocity();
  private final StatusSignal<AngularAcceleration> spinnerAcceleration = spinner.getAcceleration();
  private final StatusSignal<Voltage> spinnerVoltage = spinner.getMotorVoltage();
  private final StatusSignal<Current> spinnerStatorCurrent = spinner.getStatorCurrent();
  private final StatusSignal<Current> spinnerTorqueCurent = spinner.getTorqueCurrent();
  private final StatusSignal<Double> spinnerTargetVelocity = spinner.getClosedLoopReference();
  private final StatusSignal<Double> spinnerPIDOutput = spinner.getClosedLoopOutput();
  private final BaseStatusSignal[] spinnerSignals = {
    spinnerVelocity,
    spinnerAcceleration,
    spinnerVoltage,
    spinnerStatorCurrent,
    spinnerTorqueCurent,
    spinnerTargetVelocity,
    spinnerPIDOutput,
  };

  private final StatusSignalCollection signals = new StatusSignalCollection();

  public IndexerIOTalonFX() {
    signals.addSignals(kickerSignals);
    signals.addSignals(spinnerSignals);

    if (INDEXER_CAN_BUS == Constants.CANIVORE) {
      signals.waitForAll(1);
    }

    signals.setUpdateFrequencyForAll(Hertz.of(100));
    ParentDevice.optimizeBusUtilizationForAll(0, kicker, spinner);

    tryUntilOk(5, () -> kicker.getConfigurator().apply(INDEXER_KICKER_CONFIG, 0.25));
    tryUntilOk(5, () -> kicker.setNeutralMode(NeutralModeValue.Brake, 0.25));
    tryUntilOk(5, () -> spinner.getConfigurator().apply(INDEXER_SPINNER_CONFIG, 0.25));
    tryUntilOk(5, () -> spinner.setNeutralMode(NeutralModeValue.Brake, 0.25));
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
    signals.refreshAll();

    inputs.kickerMotorConnected = BaseStatusSignal.isAllGood(kickerSignals);
    inputs.kickerVelocity = kickerVelocity.getValue();
    inputs.kickerAcceleration = kickerAcceleration.getValue();
    inputs.kickerMotorVoltage = kickerVoltage.getValue();
    inputs.kickerStatorCurrent = kickerStatorCurrent.getValue();
    inputs.kickerTorqueCurrent = kickerTorqueCurrent.getValue();
    inputs.kickerTargetVelocity = RotationsPerSecond.of(kickerTargetVelocity.getValue());
    inputs.kickerPIDOutput = kickerPIDOutput.getValueAsDouble();

    inputs.spinnerMotorConnected = BaseStatusSignal.isAllGood(spinnerSignals);
    inputs.spinnerVelocity = spinnerVelocity.getValue();
    inputs.spinnerAcceleration = spinnerAcceleration.getValue();
    inputs.spinnerMotorVoltage = spinnerVoltage.getValue();
    inputs.spinnerStatorCurrent = spinnerStatorCurrent.getValue();
    inputs.spinnerTorqueCurrent = spinnerTorqueCurent.getValue();
    inputs.spinnerTargetVelocity = RotationsPerSecond.of(spinnerTargetVelocity.getValue());
    inputs.spinnerPIDOutput = spinnerPIDOutput.getValueAsDouble();
  }
}
