package com.frc6324.robot2026.subsystems.intake;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;
import static com.frc6324.robot2026.subsystems.intake.IntakeConstants.*;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * A hardware implementation of Intake I/O procedures.
 */
public sealed class IntakeIOTalonFX implements IntakeIO permits IntakeIOSim {
  // Hardware instances for the intake.
  protected final TalonFX deployTalon = new TalonFX(INTAKE_DEPLOY_MOTOR_ID, INTAKE_CAN_BUS);
  protected final TalonFX rollerLeaderTalon = new TalonFX(INTAKE_ROLLER_LEADER_ID, INTAKE_CAN_BUS);
  protected final TalonFX rollerFollowerTalon = new TalonFX(INTAKE_ROLLER_FOLLOWER_ID, INTAKE_CAN_BUS);

  // Control requests for the intake motors
  private final Follower followerRequest = new Follower(INTAKE_ROLLER_LEADER_ID, INTAKE_ROLLER_MOTOR_ALIGNMENT);
  private final MotionMagicTorqueCurrentFOC deployRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final MotionMagicTorqueCurrentFOC springRequest = new MotionMagicTorqueCurrentFOC(Rotations.of(1)).withSlot(1);
  private final TorqueCurrentFOC rollerRequest = new TorqueCurrentFOC(Amps.of(600));

  // Status signals
  private final StatusSignal<Angle> deployPosition = deployTalon.getPosition();
  private final StatusSignal<AngularVelocity> deployVelocity = deployTalon.getVelocity();
  private final StatusSignal<Voltage> deployMotorVoltage = deployTalon.getMotorVoltage();
  private final StatusSignal<Current> deployStatorCurrent = deployTalon.getStatorCurrent();
  private final StatusSignal<Current> deployTorqueCurrent = deployTalon.getTorqueCurrent();
  private final BaseStatusSignal[] deploySignals = {
    deployPosition, deployVelocity,
    deployMotorVoltage,
    deployStatorCurrent, deployTorqueCurrent
  };

  private final StatusSignal<AngularVelocity> rollerLeaderVelocity = rollerLeaderTalon.getVelocity();
  private final StatusSignal<AngularAcceleration> rollerLeaderAcceleration = rollerLeaderTalon.getAcceleration();
  private final StatusSignal<Voltage> rollerLeaderMotorVoltage = rollerLeaderTalon.getMotorVoltage();
  private final StatusSignal<Current> rollerLeaderStatorCurrent = rollerLeaderTalon.getStatorCurrent();
  private final StatusSignal<Current> rollerLeaderTorqueCurrent = rollerLeaderTalon.getTorqueCurrent();
  private final BaseStatusSignal[] rollerLeaderSignals = {
    rollerLeaderVelocity, rollerLeaderAcceleration,
    rollerLeaderMotorVoltage,
    rollerLeaderStatorCurrent, rollerLeaderTorqueCurrent
  };

  private final StatusSignal<AngularVelocity> rollerFollowerVelocity = rollerFollowerTalon.getVelocity();
  private final StatusSignal<AngularAcceleration> rollerFollowerAcceleration = rollerFollowerTalon.getAcceleration();
  private final StatusSignal<Voltage> rollerFollowerMotorVoltage = rollerFollowerTalon.getMotorVoltage();
  private final StatusSignal<Current> rollerFollowerStatorCurrent = rollerFollowerTalon.getStatorCurrent();
  private final StatusSignal<Current> rollerFollowerTorqueCurrent = rollerFollowerTalon.getTorqueCurrent();
  private final BaseStatusSignal[] rollerFollowerSignals = {
    rollerFollowerVelocity, rollerFollowerAcceleration,
    rollerFollowerMotorVoltage,
    rollerFollowerStatorCurrent, rollerFollowerTorqueCurrent
  };

  private final BaseStatusSignal[] signals = {
    deployPosition, deployVelocity, deployMotorVoltage, deployStatorCurrent, deployTorqueCurrent,
    rollerLeaderVelocity, rollerLeaderAcceleration, rollerLeaderMotorVoltage, rollerLeaderStatorCurrent, rollerLeaderTorqueCurrent,
    rollerFollowerVelocity, rollerFollowerAcceleration, rollerFollowerMotorVoltage, rollerFollowerStatorCurrent, rollerFollowerTorqueCurrent
  };

  /**
   * Creates an instance of I/O for a real intake.
   */
  public IntakeIOTalonFX() {
    // Set configurations for the deploy motor
    tryUntilOk(5, () -> deployTalon.getConfigurator().apply(INTAKE_DEPLOY_MOTOR_CONFIG, 0.25));
    tryUntilOk(5, () -> deployTalon.setNeutralMode(NeutralModeValue.Coast));

    // Set configurations for the roller motors
    tryUntilOk(5, () -> rollerLeaderTalon.getConfigurator().apply(INTAKE_ROLLER_MOTOR_CONFIG, 0.25));
    tryUntilOk(5, () -> rollerLeaderTalon.setNeutralMode(NeutralModeValue.Brake, 0.25));
    tryUntilOk(5, () -> rollerFollowerTalon.getConfigurator().apply(INTAKE_ROLLER_MOTOR_CONFIG, 0.25));
    tryUntilOk(5, () -> rollerFollowerTalon.setNeutralMode(NeutralModeValue.Brake));

    // Apply the follower control for the roller follower 
    tryUntilOk(5, () -> rollerFollowerTalon.setControl(followerRequest));

    if (INTAKE_CAN_BUS.isNetworkFD()) {
      // If the intake is on a CANivore, then try to synchronize all of the intake signals
      BaseStatusSignal.waitForAll(1, signals);
    }
  }

  @Override
  public void deploy() {
    deployTalon.setControl(deployRequest.withPosition(INTAKE_DEPLOYED_POSITION));
  }

  @Override
  public void runRollers() {
    rollerLeaderTalon.setControl(rollerRequest);
  }

  @Override
  public void spring() {
    deployTalon.setControl(springRequest);
  }

  @Override
  public void stopRollers() {
    rollerLeaderTalon.stopMotor();
  }

  @Override
  public void stow() {
    deployTalon.setControl(deployRequest.withPosition(INTAKE_STOWED_POSITION));
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    BaseStatusSignal.refreshAll(signals);

    inputs.deployMotorConnected = BaseStatusSignal.isAllGood(deploySignals);
    inputs.rollerLeaderConnected = BaseStatusSignal.isAllGood(rollerLeaderSignals);
    inputs.rollerFollowerConnected = BaseStatusSignal.isAllGood(rollerFollowerSignals);

    inputs.deployPosition = deployPosition.getValue();
    inputs.deployMotorVelocity = deployVelocity.getValue();
    inputs.deployVoltage = deployMotorVoltage.getValue();
    inputs.deployStatorCurrent = deployStatorCurrent.getValue();
    inputs.deployTorqueCurrent = deployTorqueCurrent.getValue();

    inputs.rollerLeaderVelocity = rollerLeaderVelocity.getValue();
    inputs.rollerLeaderAcceleration = rollerLeaderAcceleration.getValue();
    inputs.rollerLeaderVoltage = rollerLeaderMotorVoltage.getValue();
    inputs.rollerLeaderStatorCurrent = rollerLeaderStatorCurrent.getValue();
    inputs.rollerLeaderTorqueCurrent = rollerLeaderTorqueCurrent.getValue();

    inputs.rollerFollowerVelocity = rollerFollowerVelocity.getValue();
    inputs.rollerFollowerAcceleration = rollerFollowerAcceleration.getValue();
    inputs.rollerFollowerVoltage = rollerFollowerMotorVoltage.getValue();
    inputs.rollerFollowerStatorCurrent = rollerFollowerStatorCurrent.getValue();
    inputs.rollerFollowerTorqueCurrent = rollerFollowerTorqueCurrent.getValue();
  }
}
