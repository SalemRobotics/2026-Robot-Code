package com.frc6324.robot2026.subsystems.climber;

import static com.frc6324.robot2026.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.frc6324.lib.util.PhoenixUtil;
import edu.wpi.first.units.measure.*;

/** A real-world implementation of the climber's I/O using a TalonFX motor controller. */
public class ClimberIOTalonFX implements ClimberIO {
  // TODO: determine if the climber needs a follower for performance reasons
  protected final TalonFX talon = new TalonFX(CLIMBER_MOTOR_ID, CLIMBER_CAN_BUS);

  private final DutyCycleOut output =
      new DutyCycleOut(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);

  private final StatusSignal<AngularVelocity> motorVelocity = talon.getVelocity();
  private final StatusSignal<Voltage> motorVoltage = talon.getMotorVoltage();
  private final StatusSignal<Current> motorStatorCurrent = talon.getStatorCurrent();
  private final StatusSignal<Current> motorSupplyCurrent = talon.getSupplyCurrent();
  private final StatusSignal<Current> motorTorqueCurrent = talon.getTorqueCurrent();
  private final StatusSignalCollection signals =
      new StatusSignalCollection(
          motorVelocity, motorVoltage, motorStatorCurrent, motorSupplyCurrent, motorTorqueCurrent);

  /** Creates an I/O implementation for the climber backed by a TalonFX motor controller. */
  public ClimberIOTalonFX() {
    PhoenixUtil.tryUntilOk(
        5,
        () -> talon.getConfigurator().apply(CLIMBER_CURRENT_LIMITS, 0.25),
        "Failed to configure the climber motor's current limits");
    PhoenixUtil.tryUntilOk(
        5,
        () -> talon.setNeutralMode(NeutralModeValue.Brake),
        "Failed to configure the climber motor's neutral mode.");
  }

  @Override
  public void deploy() {
    talon.setControl(output.withOutput(CLIMBER_DEPLOY_OUTPUT_PERCENTAGE));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void stow() {
    talon.setControl(output.withOutput(CLIMBER_STOW_OUTPUT_PERCENTAGE));
  }

  @Override
  public void updateInputs(ClimberInputs inputs) {
    // Check if the motor is connected and OK
    inputs.motorConnected = signals.refreshAll().isOK();

    // Update generic motor info
    inputs.motorVelocity = motorVelocity.getValue();
    inputs.motorVoltage = motorVoltage.getValue();

    // Update current values
    inputs.statorCurrent = motorStatorCurrent.getValue();
    inputs.supplyCurrent = motorSupplyCurrent.getValue();
    inputs.torqueCurrent = motorTorqueCurrent.getValue();
  }
}
