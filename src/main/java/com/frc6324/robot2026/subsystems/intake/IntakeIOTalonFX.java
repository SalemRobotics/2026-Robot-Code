package com.frc6324.robot2026.subsystems.intake;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;
import static com.frc6324.robot2026.subsystems.intake.IntakeConstants.*;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public sealed class IntakeIOTalonFX implements IntakeIO permits IntakeIOSim {
  protected final TalonFX extensionTalon = new TalonFX(EXTENSION_MOTOR_ID, INTAKE_CAN_BUS);
  protected final TalonFX spinLeaderTalon = new TalonFX(SPIN_LEADER_MOTOR_ID, INTAKE_CAN_BUS);
  protected final TalonFX spinFollowerTalon = new TalonFX(SPIN_FOLLOWER_MOTOR_ID, INTAKE_CAN_BUS);

  private final MotionMagicTorqueCurrentFOC extensionRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final MotionMagicTorqueCurrentFOC springRequest =
      new MotionMagicTorqueCurrentFOC(Rotations.of(1)).withSlot(1);

  private final TorqueCurrentFOC spinRequest = new TorqueCurrentFOC(Amps.of(600));

  public IntakeIOTalonFX() {
    tryUntilOk(5, () -> extensionTalon.getConfigurator().apply(EXTENSION_MOTOR_CONFIG, 0.25));
    tryUntilOk(5, () -> extensionTalon.setPosition(0, 0.25));
    tryUntilOk(5, () -> extensionTalon.setNeutralMode(NeutralModeValue.Coast));

    tryUntilOk(5, () -> spinLeaderTalon.getConfigurator().apply(SPIN_LEADER_MOTOR_CONFIG, 0.25));
    tryUntilOk(5, () -> spinLeaderTalon.setPosition(0, 0.25));
    tryUntilOk(5, () -> spinLeaderTalon.setNeutralMode(NeutralModeValue.Brake, 0.25));

    final Follower followerRequest =
        new Follower(SPIN_LEADER_MOTOR_ID, MotorAlignmentValue.Opposed)
            .withUpdateFreqHz(Hertz.of(1000));
    tryUntilOk(
        5, () -> spinFollowerTalon.getConfigurator().apply(SPIN_FOLLOWER_MOTOR_CONFIG, 0.25));
    tryUntilOk(5, () -> spinFollowerTalon.setPosition(0, 0.25));
    tryUntilOk(5, () -> spinFollowerTalon.setNeutralMode(NeutralModeValue.Brake));
    tryUntilOk(5, () -> spinFollowerTalon.setControl(followerRequest));
  }

  @Override
  public void deploy() {
    extensionTalon.setControl(extensionRequest.withPosition(1));
  }

  @Override
  public void runRollers() {
    spinLeaderTalon.setControl(spinRequest);
  }

  @Override
  public void spring() {
    extensionTalon.setControl(springRequest);
  }

  @Override
  public void stopRollers() {
    spinLeaderTalon.stopMotor();
  }

  @Override
  public void stow() {
    extensionTalon.setControl(extensionRequest.withPosition(0));
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    // TODO: implement this !
  }
}
