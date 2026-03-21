package com.frc6324.robot2026.subsystems.shooter;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.*;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.FlywheelConstants.*;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.HoodConstants.*;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.frc6324.lib.util.PhoenixUtil;
import edu.wpi.first.units.measure.*;

public class ShooterIOTalonFX implements ShooterIO {
  protected final TalonFX hoodTalon = new TalonFX(HOOD_MOTOR_ID, SHOOTER_CAN_BUS);
  protected final TalonFX flywheelLeader = new TalonFX(FLYWHEEL_LEADER_ID, SHOOTER_CAN_BUS);
  protected final TalonFX flywheelFollower = new TalonFX(FLYWHEEL_FOLLOWER_ID, SHOOTER_CAN_BUS);

  // Control requests (have higher update frequencies to make PID smoother-ish and stick faster to )
  private final PositionTorqueCurrentFOC hoodRequest =
      new PositionTorqueCurrentFOC(0)
          .withSlot(0)
          .withUpdateFreqHz(Hertz.of(500))
          .withUseTimesync(true);

  private final VelocityTorqueCurrentFOC flywheelRequest =
      new VelocityTorqueCurrentFOC(0)
          .withSlot(0)
          .withUpdateFreqHz(Hertz.of(1000))
          .withUseTimesync(true);

  private final Follower followerRequest =
      new Follower(FLYWHEEL_LEADER_ID, FLYWHEEL_MOTOR_ALIGNMENT).withUpdateFreqHz(Hertz.of(1000));

  private final StatusSignal<Angle> hoodPosition = hoodTalon.getPosition();
  private final StatusSignal<AngularVelocity> hoodVelocity = hoodTalon.getVelocity();
  private final StatusSignal<Voltage> hoodMotorVoltage = hoodTalon.getMotorVoltage();
  private final StatusSignal<Current> hoodStatorCurrent = hoodTalon.getStatorCurrent();
  private final BaseStatusSignal[] hoodSignals = {
    hoodPosition, hoodVelocity, hoodMotorVoltage, hoodStatorCurrent
  };

  private final StatusSignal<AngularVelocity> flywheelVelocity = flywheelLeader.getVelocity();
  private final StatusSignal<Voltage> flywheelMotorVoltage = flywheelLeader.getMotorVoltage();
  private final StatusSignal<Current> flywheelStatorCurrent = flywheelLeader.getStatorCurrent();
  private final BaseStatusSignal[] flywheelLeaderSignals = {
    flywheelVelocity, flywheelMotorVoltage, flywheelStatorCurrent,
  };

  private final StatusSignal<AngularVelocity> flywheelFollowerVelocity =
      flywheelFollower.getVelocity();
  private final StatusSignal<Voltage> flywheelFollowerMotorVoltage =
      flywheelFollower.getMotorVoltage();
  private final StatusSignal<Current> flywheelFollowerStatorCurrent =
      flywheelFollower.getStatorCurrent();
  private final BaseStatusSignal[] flywheelFollowerSignals = {
    flywheelFollowerVelocity, flywheelFollowerMotorVoltage, flywheelFollowerStatorCurrent,
  };

  public ShooterIOTalonFX() {
    tryUntilOk(5, () -> hoodTalon.getConfigurator().apply(HOOD_MOTOR_CONFIG));

    // Apply the config to the leader
    tryUntilOk(5, () -> flywheelLeader.getConfigurator().apply(FLYWHEEL_MOTOR_CONFIG));

    // Invert the config & apply to the follower
    FLYWHEEL_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> flywheelFollower.getConfigurator().apply(FLYWHEEL_MOTOR_CONFIG));

    PhoenixUtil.addSignals(hoodTalon, hoodSignals);
    PhoenixUtil.addSignals(flywheelLeader, flywheelLeaderSignals);
    PhoenixUtil.addSignals(flywheelFollower, flywheelFollowerSignals);
    ParentDevice.optimizeBusUtilizationForAll(0, hoodTalon, flywheelLeader, flywheelFollower);
  }

  @Override
  public void coastFlywheel() {
    flywheelLeader.stopMotor();
    flywheelFollower.stopMotor();
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity velocity, int slot) {
    flywheelLeader.setControl(flywheelRequest.withVelocity(velocity).withSlot(slot));
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
    inputs.hoodConnected = BaseStatusSignal.isAllGood(hoodSignals);
    inputs.flywheelLeaderConnected = BaseStatusSignal.isAllGood(flywheelLeaderSignals);
    inputs.flywheelFollowerConnected = BaseStatusSignal.isAllGood(flywheelFollowerSignals);

    inputs.hoodPosition = hoodPosition.getValue();
    inputs.hoodVelocity = hoodVelocity.getValue();
    inputs.hoodMotorVoltage = hoodMotorVoltage.getValue();
    inputs.hoodStatorCurrent = hoodStatorCurrent.getValue();

    inputs.flywheelLeaderVelocity = flywheelVelocity.getValue();
    inputs.flywheelLeaderMotorVoltage = flywheelMotorVoltage.getValue();
    inputs.flywheelLeaderStatorCurrent = flywheelStatorCurrent.getValue();

    inputs.flywheelFollowerVelocity = flywheelFollowerVelocity.getValue();
    inputs.flywheelFollowerMotorVoltage = flywheelFollowerMotorVoltage.getValue();
    inputs.flywheelFollowerStatorCurrent = flywheelFollowerStatorCurrent.getValue();
  }
}
