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
    // TODO: implement this!
  }
}
