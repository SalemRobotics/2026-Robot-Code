package com.frc6324.robot2026.subsystems.shooter;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.*;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.FlywheelConstants.*;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.HoodConstants.*;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

public class ShooterIOTalonFX implements ShooterIO {
  protected final TalonFX hoodTalon = new TalonFX(HOOD_MOTOR_ID, SHOOTER_CAN_BUS);
  protected final TalonFX flywheelLeader = new TalonFX(FLYWHEEL_LEADER_ID, SHOOTER_CAN_BUS);
  protected final TalonFX flywheelFollower = new TalonFX(FLYWHEEL_FOLLOWER_ID, SHOOTER_CAN_BUS);

  private final MotionMagicTorqueCurrentFOC hoodRequest = new MotionMagicTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC flywheelRequest = new VelocityTorqueCurrentFOC(0);
  private final Follower followerRequest =
      new Follower(FLYWHEEL_LEADER_ID, FLYWHEEL_MOTOR_ALIGNMENT);

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

  private final StatusSignal<AngularVelocity> flywheelVelocity = flywheelLeader.getVelocity();
  private final StatusSignal<AngularAcceleration> flywheelAcceleration =
      flywheelLeader.getAcceleration();
  private final StatusSignal<Double> flywheelPIDSetpoint = flywheelLeader.getClosedLoopReference();
  private final StatusSignal<Double> flywheelPIDOutput = flywheelLeader.getClosedLoopOutput();
  private final StatusSignal<Voltage> flywheelMotorVoltage = flywheelLeader.getMotorVoltage();
  private final StatusSignal<Current> flywheelStatorCurrent = flywheelLeader.getStatorCurrent();
  private final StatusSignal<Current> flywheelTorqueCurrent = flywheelLeader.getTorqueCurrent();
  private final BaseStatusSignal[] flywheelSignals = {
    flywheelVelocity,
    flywheelAcceleration,
    flywheelPIDSetpoint,
    flywheelPIDOutput,
    flywheelMotorVoltage,
    flywheelStatorCurrent,
    flywheelTorqueCurrent
  };

  private final StatusSignal<AngularVelocity> flywheelFollowerVelocity =
      flywheelFollower.getVelocity();
  private final StatusSignal<AngularAcceleration> flywheelFollowerAcceleration =
      flywheelFollower.getAcceleration();
  private final StatusSignal<Voltage> flywheelFollowerMotorVoltage =
      flywheelFollower.getMotorVoltage();
  private final StatusSignal<Current> flywheelFollowerStatorCurrent =
      flywheelFollower.getStatorCurrent();
  private final StatusSignal<Current> flywheelFollowerTorqueCurrent =
      flywheelFollower.getTorqueCurrent();
  private final BaseStatusSignal[] flywheelFollowerSignals = {
    flywheelFollowerVelocity,
    flywheelFollowerAcceleration,
    flywheelFollowerMotorVoltage,
    flywheelFollowerStatorCurrent,
    flywheelFollowerTorqueCurrent
  };

  private final StatusSignalCollection signals = new StatusSignalCollection();

  public ShooterIOTalonFX() {
    signals.addSignals(hoodSignals);
    signals.addSignals(flywheelSignals);
    signals.addSignals(flywheelFollowerSignals);

    tryUntilOk(5, () -> hoodTalon.getConfigurator().apply(HOOD_MOTOR_CONFIG));
    tryUntilOk(5, () -> hoodTalon.setNeutralMode(NeutralModeValue.Brake));

    tryUntilOk(5, () -> flywheelLeader.getConfigurator().apply(FLYWHEEL_MOTOR_CONFIG));
    tryUntilOk(5, () -> flywheelLeader.setNeutralMode(NeutralModeValue.Coast));
    tryUntilOk(5, () -> flywheelFollower.getConfigurator().apply(FLYWHEEL_MOTOR_CONFIG));
    tryUntilOk(5, () -> flywheelFollower.setNeutralMode(NeutralModeValue.Coast));

    signals.setUpdateFrequencyForAll(Hertz.of(100));
    ParentDevice.optimizeBusUtilizationForAll(0, hoodTalon, flywheelLeader, flywheelFollower);
  }

  @Override
  public void coastFlywheel() {
    flywheelLeader.stopMotor();
    flywheelFollower.stopMotor();
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity velocity) {
    flywheelLeader.setControl(flywheelRequest.withVelocity(velocity));
    flywheelFollower.setControl(followerRequest);
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
    inputs.flywheelLeaderConnected = BaseStatusSignal.isAllGood(flywheelSignals);
    inputs.flywheelFollowerConnected = BaseStatusSignal.isAllGood(flywheelFollowerSignals);

    inputs.hoodPosition = hoodPosition.getValue();
    inputs.hoodVelocity = hoodVelocity.getValue();
    inputs.hoodPIDSetpoint = hoodPIDSetpoint.getValueAsDouble();
    inputs.hoodPIDOutput = hoodPIDOutput.getValueAsDouble();
    inputs.hoodMotorVoltage = hoodMotorVoltage.getValue();
    inputs.hoodStatorCurrent = hoodStatorCurrent.getValue();
    inputs.hoodTorqueCurrent = hoodTorqueCurrent.getValue();

    inputs.flywheelLeaderVelocity = flywheelVelocity.getValue();
    inputs.flywheelLeaderAcceleration = flywheelAcceleration.getValue();
    inputs.flywheelLeaderPIDSetpoint = flywheelPIDSetpoint.getValueAsDouble();
    inputs.flywheelLeaderPIDOutput = flywheelPIDOutput.getValueAsDouble();
    inputs.flywheelLeaderMotorVoltage = flywheelMotorVoltage.getValue();
    inputs.flywheelLeaderStatorCurrent = flywheelStatorCurrent.getValue();
    inputs.flywheelLeaderTorqueCurrent = flywheelTorqueCurrent.getValue();

    inputs.flywheelFollowerVelocity = flywheelFollowerVelocity.getValue();
    inputs.flywheelFollowerAcceleration = flywheelFollowerAcceleration.getValue();
    inputs.flywheelFollowerMotorVoltage = flywheelFollowerMotorVoltage.getValue();
    inputs.flywheelFollowerStatorCurrent = flywheelFollowerStatorCurrent.getValue();
    inputs.flywheelFollowerTorqueCurrent = flywheelFollowerTorqueCurrent.getValue();
  }
}
