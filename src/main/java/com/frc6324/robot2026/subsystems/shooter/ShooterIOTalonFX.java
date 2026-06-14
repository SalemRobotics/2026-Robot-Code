package com.frc6324.robot2026.subsystems.shooter;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.AcceleratorConstants.*;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.DrumConstants.*;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.HoodConstants.*;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.SHOOTER_CAN_BUS;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;

public class ShooterIOTalonFX implements ShooterIO {
  protected final TalonFX acceleratorTalon = new TalonFX(ACCELERATOR_MOTOR_ID, SHOOTER_CAN_BUS);
  protected final TalonFX drumLeader = new TalonFX(DRUM_LEADER_ID, SHOOTER_CAN_BUS);
  private final TalonFX[] drumFollowers = new TalonFX[DRUM_FOLLOWER_DIRECTIONS.length];
  protected final TalonFX hoodTalon = new TalonFX(HOOD_MOTOR_ID, SHOOTER_CAN_BUS);

  private final StatusSignal<AngularVelocity> acceleratorVelocity = acceleratorTalon.getVelocity();
  private final StatusSignal<Voltage> acceleratorMotorVoltage = acceleratorTalon.getMotorVoltage();
  private final StatusSignal<Current> acceleratorStatorCurrent =
      acceleratorTalon.getStatorCurrent();

  private final StatusSignal<AngularVelocity> drumVelocity = drumLeader.getVelocity();
  private final StatusSignal<Voltage> drumMotorVoltage = drumLeader.getMotorVoltage();
  private final StatusSignal<Current> drumStatorCurrent = drumLeader.getStatorCurrent();

  private final StatusSignalCollection[] followerSignalCollections =
      new StatusSignalCollection[drumFollowers.length];

  private final StatusSignal<Angle> hoodPosition = hoodTalon.getPosition();
  private final StatusSignal<Voltage> hoodMotorVoltage = hoodTalon.getMotorVoltage();
  private final StatusSignal<Current> hoodStatorCurrent = hoodTalon.getStatorCurrent();

  private final PositionVoltage hoodRequest =
      new PositionVoltage(0)
          .withEnableFOC(true)
          .withOverrideBrakeDurNeutral(true)
          .withUseTimesync(true)
          .withSlot(0);
  private final VelocityTorqueCurrentFOC drumLeaderRequest =
      new VelocityTorqueCurrentFOC(0)
          .withOverrideCoastDurNeutral(true)
          .withUpdateFreqHz(Hertz.of(1000))
          .withUseTimesync(true)
          .withSlot(0);
  private final VelocityTorqueCurrentFOC acceleratorRequest =
      new VelocityTorqueCurrentFOC(0)
          .withOverrideCoastDurNeutral(true)
          .withUseTimesync(true)
          .withSlot(0);
  private final StrictFollower followerRequest =
      new StrictFollower(DRUM_LEADER_ID).withUpdateFreqHz(drumLeaderRequest.UpdateFreqHz);

  @SuppressWarnings("resource")
  public ShooterIOTalonFX() {
    tryUntilOk(5, () -> acceleratorTalon.getConfigurator().apply(ACCELERATOR_MOTOR_CONFIG));
    tryUntilOk(5, () -> hoodTalon.getConfigurator().apply(HOOD_MOTOR_CONFIG));

    // Set output signals to 1KHz for followers
    drumLeader.getMotorVoltage().setUpdateFrequency(Hertz.of(1000));
    drumLeader.getTorqueCurrent().setUpdateFrequency(Hertz.of(1000));

    tryUntilOk(5, () -> drumLeader.getConfigurator().apply(DRUM_MOTOR_CONFIG));

    for (int i = 0; i < drumFollowers.length; i++) {
      // talon isn't actually leaked here but java likes to complain :/
      final TalonFX talon = new TalonFX(DRUM_MOTOR_IDS[i], SHOOTER_CAN_BUS);

      final TalonFXConfiguration config = DRUM_MOTOR_CONFIG;
      config.MotorOutput.Inverted = DRUM_FOLLOWER_DIRECTIONS[i];

      tryUntilOk(5, () -> talon.getConfigurator().apply(config));
      tryUntilOk(5, () -> talon.setControl(followerRequest));

      drumFollowers[i] = talon;
      final StatusSignalCollection signals =
          new StatusSignalCollection(talon.getMotorVoltage(), talon.getStatorCurrent());
      followerSignalCollections[i] = signals;

      signals.setUpdateFrequencyForAll(Hertz.of(100));
      talon.optimizeBusUtilization();
    }
  }

  @Override
  public void coastDrum() {
    drumLeader.stopMotor();
  }

  @Override
  public void setAcceleratorVelocity(AngularVelocity velocity) {
    acceleratorTalon.setControl(acceleratorRequest.withVelocity(velocity));
  }

  @Override
  public void setDrumVelocity(AngularVelocity velocity, int slot) {
    drumLeader.setControl(drumLeaderRequest.withVelocity(velocity).withSlot(slot));
  }

  @Override
  public void setHoodPosition(Angle position) {
    hoodTalon.setControl(hoodRequest.withPosition(position).withSlot(0));
  }

  @Override
  public void stopAcceleratorMotor() {
    acceleratorTalon.stopMotor();
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.acceleratorConnected =
        BaseStatusSignal.refreshAll(
                acceleratorVelocity, acceleratorMotorVoltage, acceleratorStatorCurrent)
            .isOK();
    inputs.acceleratorVelocity = acceleratorVelocity.getValue();
    inputs.acceleratorMotorVoltage = acceleratorMotorVoltage.getValue();
    inputs.acceleratorStatorCurrent = acceleratorStatorCurrent.getValue();

    inputs.hoodConnected =
        BaseStatusSignal.refreshAll(hoodPosition, hoodMotorVoltage, hoodStatorCurrent).isOK();
    inputs.hoodPosition = hoodPosition.getValue();
    inputs.hoodMotorVoltage = hoodMotorVoltage.getValue();
    inputs.hoodStatorCurrent = hoodStatorCurrent.getValue();

    inputs.drumMotorsConnected = new boolean[4];

    inputs.drumMotorsConnected[0] =
        BaseStatusSignal.refreshAll(drumVelocity, drumMotorVoltage, drumStatorCurrent).isOK();
    inputs.drumVelocity = drumVelocity.getValue();
    inputs.drumMotorVoltages[0] = drumMotorVoltage.getValueAsDouble();
    inputs.drumStatorCurrents[0] = drumStatorCurrent.getValueAsDouble();
    for (int i = 0; i < drumFollowers.length; i++) {
      final int motor = i + 1;
      final TalonFX talon = drumFollowers[i];

      inputs.drumMotorsConnected[motor] = followerSignalCollections[i].refreshAll().isOK();
      inputs.drumMotorVoltages[motor] = talon.getMotorVoltage().getValueAsDouble();
      inputs.drumStatorCurrents[motor] = talon.getStatorCurrent().getValueAsDouble();
    }
  }
}
