package com.frc6324.robot2026.subsystems.rollers;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;
import static com.frc6324.robot2026.subsystems.rollers.RollerConstants.*;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.units.measure.*;

public class RollerIOTalonFX implements RollerIO {
  protected final TalonFX leader = new TalonFX(ROLLER_LEADER_ID, ROLLER_CAN_BUS);
  protected final TalonFX follower = new TalonFX(ROLLER_FOLLOWER_ID, ROLLER_CAN_BUS);

  private final Follower followerRequest = new Follower(ROLLER_LEADER_ID, ROLLER_ALIGNMENT);
  private final TorqueCurrentFOC spinRequest = new TorqueCurrentFOC(ROLLER_SPIN_CURRENT);
  private final CoastOut coast = new CoastOut();

  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<AngularAcceleration> leaderAcceleration = leader.getAcceleration();
  private final StatusSignal<Voltage> leaderMotorVoltage = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> leaderTorqueCurrent = leader.getTorqueCurrent();
  private final BaseStatusSignal[] leaderSignals = {
    leaderVelocity, leaderAcceleration, leaderMotorVoltage, leaderStatorCurrent, leaderTorqueCurrent
  };

  private final StatusSignal<AngularVelocity> followerVelocity = follower.getVelocity();
  private final StatusSignal<AngularAcceleration> followerAcceleration = follower.getAcceleration();
  private final StatusSignal<Voltage> followerMotorVoltage = follower.getMotorVoltage();
  private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent();
  private final StatusSignal<Current> followerTorqueCurrent = follower.getTorqueCurrent();
  private final BaseStatusSignal[] followerSignals = {
    followerVelocity,
    followerAcceleration,
    followerMotorVoltage,
    followerStatorCurrent,
    followerTorqueCurrent
  };

  private final StatusSignalCollection rollerSignals = new StatusSignalCollection();

  public RollerIOTalonFX() {
    // Add all of the leader and follower's signals to the larger collection
    rollerSignals.addSignals(leaderSignals);
    rollerSignals.addSignals(followerSignals);

    if (ROLLER_CAN_BUS == Constants.CANIVORE) {
      // If the rollers are on the CANivore, synchronize all of its signals
      rollerSignals.waitForAll(1);
    }

    // Set the update frequency to that of the robot code
    rollerSignals.setUpdateFrequencyForAll(Hertz.of(50));
    // Disable all unneccesary status signals to save CAN bus utilization
    ParentDevice.optimizeBusUtilizationForAll(0, leader, follower);

    // Configure both of the motors
    tryUntilOk(5, () -> leader.getConfigurator().apply(ROLLER_MOTOR_CONFIG, 0.25));
    tryUntilOk(5, () -> follower.getConfigurator().apply(ROLLER_MOTOR_CONFIG, 0.25));

    // Configure the motors to brake when stopped
    tryUntilOk(5, () -> leader.setNeutralMode(NeutralModeValue.Brake, 0.25));
    tryUntilOk(5, () -> follower.setNeutralMode(NeutralModeValue.Brake, 0.25));
  }

  @Override
  public void coast() {
    leader.setControl(coast);
    follower.setControl(coast);
  }

  @Override
  public void start() {
    leader.setControl(spinRequest);
    follower.setControl(followerRequest);
  }

  @Override
  public void stop() {
    leader.stopMotor();
    follower.stopMotor();
  }

  @Override
  public void updateInputs(RollerInputs inputs) {
    rollerSignals.refreshAll();

    inputs.leaderConnected = BaseStatusSignal.isAllGood(leaderSignals);
    inputs.followerConnected = BaseStatusSignal.isAllGood(followerSignals);

    inputs.leaderVelocity = leaderVelocity.getValue();
    inputs.leaderAcceleration = leaderAcceleration.getValue();
    inputs.leaderMotorVoltage = leaderMotorVoltage.getValue();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValue();
    inputs.leaderTorqueCurrent = leaderTorqueCurrent.getValue();

    inputs.followerVelocity = followerVelocity.getValue();
    inputs.followerAcceleration = followerAcceleration.getValue();
    inputs.followerMotorVoltage = followerMotorVoltage.getValue();
    inputs.followerStatorCurrent = followerStatorCurrent.getValue();
    inputs.followerTorqueCurrent = followerTorqueCurrent.getValue();
  }
}
