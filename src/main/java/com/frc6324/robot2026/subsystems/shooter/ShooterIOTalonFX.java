package com.frc6324.robot2026.subsystems.shooter;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.*;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;

public class ShooterIOTalonFX implements ShooterIO {
  protected final TalonFX hoodTalon = new TalonFX(SHOOTER_HOOD_MOTOR_ID, SHOOTER_CAN_BUS);
  protected final TalonFX flywheelTalon = new TalonFX(SHOOTER_FLYWHEEL_MOTOR_ID, SHOOTER_CAN_BUS);

  private final MotionMagicTorqueCurrentFOC hoodRequest = new MotionMagicTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC flywheelRequest = new VelocityTorqueCurrentFOC(0);

  private final StatusSignal<Angle> hoodPosition = hoodTalon.getPosition();
  private final StatusSignal<AngularVelocity> hoodVelocity = hoodTalon.getVelocity();
  private final StatusSignal<Double> hoodPIDSetpoint = hoodTalon.getClosedLoopReference();
  private final StatusSignal<Double> hoodPIDOutput = hoodTalon.getClosedLoopOutput();
  private final StatusSignal<Voltage> hoodMotorVoltage = hoodTalon.getMotorVoltage();
  private final StatusSignal<Current> hoodStatorCurrent = hoodTalon.getStatorCurrent();
  private final StatusSignal<Current> hoodTorqueCurrent = hoodTalon.getTorqueCurrent();
  private final BaseStatusSignal[] hoodSignals = {
    hoodPosition,
    hoodVelocity,
    hoodPIDSetpoint,
    hoodPIDOutput,
    hoodMotorVoltage,
    hoodStatorCurrent,
    hoodTorqueCurrent
  };

  private final StatusSignal<AngularVelocity> flywheelVelocity = flywheelTalon.getVelocity();
  private final StatusSignal<AngularAcceleration> flywheelAcceleration =
      flywheelTalon.getAcceleration();
  private final StatusSignal<Double> flywheelPIDSetpoint = flywheelTalon.getClosedLoopReference();
  private final StatusSignal<Double> flywheelPIDOutput = flywheelTalon.getClosedLoopOutput();
  private final StatusSignal<Voltage> flywheelMotorVoltage = flywheelTalon.getMotorVoltage();
  private final StatusSignal<Current> flywheelStatorCurrent = flywheelTalon.getStatorCurrent();
  private final StatusSignal<Current> flywheelTorqueCurrent = flywheelTalon.getTorqueCurrent();
  private final BaseStatusSignal[] flywheelSignals = {
    flywheelVelocity,
    flywheelAcceleration,
    flywheelPIDSetpoint,
    flywheelPIDOutput,
    flywheelMotorVoltage,
    flywheelStatorCurrent,
    flywheelTorqueCurrent
  };

  private final StatusSignalCollection signals = new StatusSignalCollection();

  public ShooterIOTalonFX() {
    signals.addSignals(hoodSignals);
    signals.addSignals(flywheelSignals);

    tryUntilOk(5, () -> hoodTalon.getConfigurator().apply(SHOOTER_HOOD_MOTOR_CONFIG));
    tryUntilOk(5, () -> hoodTalon.setNeutralMode(NeutralModeValue.Brake));

    tryUntilOk(5, () -> flywheelTalon.getConfigurator().apply(SHOOTER_FLYWHEEL_MOTOR_CONFIG));
    tryUntilOk(5, () -> flywheelTalon.setNeutralMode(NeutralModeValue.Coast));

    if (RobotBase.isReal() && SHOOTER_CAN_BUS == Constants.CANIVORE) {
      signals.waitForAll(1);
    }

    signals.setUpdateFrequencyForAll(Hertz.of(100));
    ParentDevice.optimizeBusUtilizationForAll(0, hoodTalon, flywheelTalon);
  }

  @Override
  public void coastFlywheel() {
    flywheelTalon.stopMotor();
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity velocity) {
    flywheelTalon.setControl(flywheelRequest.withVelocity(velocity));
  }

  @Override
  public void setHoodAngle(Angle angle) {
    hoodTalon.setControl(hoodRequest.withPosition(angle));
  }

  @Override
  public void stopHoodMotor() {
    hoodTalon.stopMotor();
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    signals.refreshAll();

    inputs.hoodConnected = BaseStatusSignal.isAllGood(hoodSignals);
    inputs.flywheelConnected = BaseStatusSignal.isAllGood(flywheelSignals);

    inputs.hoodPosition = hoodPosition.getValue();
    inputs.hoodVelocity = hoodVelocity.getValue();
    inputs.hoodPIDSetpoint = hoodPIDSetpoint.getValueAsDouble();
    inputs.hoodPIDOutput = hoodPIDOutput.getValueAsDouble();
    inputs.hoodMotorVoltage = hoodMotorVoltage.getValue();
    inputs.hoodStatorCurrent = hoodStatorCurrent.getValue();
    inputs.hoodTorqueCurrent = hoodTorqueCurrent.getValue();

    inputs.flywheelVelocity = flywheelVelocity.getValue();
    inputs.flywheelAcceleration = flywheelAcceleration.getValue();
    inputs.flywheelPIDSetpoint = flywheelPIDSetpoint.getValueAsDouble();
    inputs.flywheelPIDOutput = flywheelPIDOutput.getValueAsDouble();
    inputs.flywheelMotorVoltage = flywheelMotorVoltage.getValue();
    inputs.flywheelStatorCurrent = flywheelStatorCurrent.getValue();
    inputs.flywheelTorqueCurrent = flywheelTorqueCurrent.getValue();
  }
}
