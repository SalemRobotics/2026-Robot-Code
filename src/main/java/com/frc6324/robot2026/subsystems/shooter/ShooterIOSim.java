package com.frc6324.robot2026.subsystems.shooter;

import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.AcceleratorConstants.*;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.DrumConstants.*;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.HoodConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.frc6324.lib.util.DeltaTimeCalculator;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public final class ShooterIOSim extends ShooterIOTalonFX {
  private final TalonFXSimState acceleratorSimState = acceleratorTalon.getSimState();
  private final DCMotorSim acceleratorSimulation =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              ACCELERATOR_GEARBOX, ACCELERATOR_MOI, ACCELERATOR_REDUCTION),
          ACCELERATOR_GEARBOX);

  private final TalonFXSimState drumSimState = drumLeader.getSimState();
  private final DCMotorSim drumSimulation =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(DRUM_GEARBOX, DRUM_MOI, 1.0), DRUM_GEARBOX);

  private final TalonFXSimState hoodSimState = hoodTalon.getSimState();
  private final DCMotorSim hoodSimulation =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              HOOD_GEARBOX, HOOD_MOI, HOOD_ROTOR_TO_ENCODER * HOOD_ENCODER_TO_MECHANISM),
          HOOD_GEARBOX);

  private final DeltaTimeCalculator deltaTime = new DeltaTimeCalculator();

  /** Creates a new sim implementation of the shooter. */
  public ShooterIOSim() {
    acceleratorSimState.setMotorType(ACCELERATOR_MOTOR_TYPE);
    drumSimState.setMotorType(DRUM_MOTOR_TYPE);
    hoodSimState.setMotorType(HOOD_MOTOR_TYPE);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    // Get the time since the last loop and the robot's current battery
    final double dt = deltaTime.get();
    final double batteryVoltage = RobotController.getBatteryVoltage();

    // Set the supply voltages for all of the sim states
    acceleratorSimState.setSupplyVoltage(batteryVoltage);
    drumSimState.setSupplyVoltage(batteryVoltage);
    hoodSimState.setSupplyVoltage(batteryVoltage);

    // Update the accelerator motor sim
    acceleratorSimulation.setInputVoltage(acceleratorSimState.getMotorVoltage());
    acceleratorSimulation.update(dt);

    // Update the drum motor sim. Note that this only drives the leader's sim
    // state; the three follower TalonFXs (private to ShooterIOTalonFX) are not
    // simulated here, so their connectivity/voltage/current signals will not
    // reflect closed-loop sim behavior.
    drumSimulation.setInputVoltage(drumSimState.getMotorVoltage());
    drumSimulation.update(dt);

    // Update the hood motor sim
    hoodSimulation.setInputVoltage(hoodSimState.getMotorVoltage());
    hoodSimulation.update(dt);

    // Set the phoenix sim state for the accelerator motor
    acceleratorSimState.setRawRotorPosition(
        acceleratorSimulation.getAngularPosition().times(ACCELERATOR_REDUCTION));
    acceleratorSimState.setRotorVelocity(
        acceleratorSimulation.getAngularVelocity().times(ACCELERATOR_REDUCTION));
    acceleratorSimState.setRotorAcceleration(
        acceleratorSimulation.getAngularAcceleration().times(ACCELERATOR_REDUCTION));

    // Set the phoenix sim state for the drum leader motor
    drumSimState.setRawRotorPosition(drumSimulation.getAngularPosition());
    drumSimState.setRotorVelocity(drumSimulation.getAngularVelocity());
    drumSimState.setRotorAcceleration(drumSimulation.getAngularAcceleration());

    // Set the phoenix sim state for the hood motor. The TalonFX is configured
    // with a fused CANcoder and a rotor->sensor->mechanism ratio, so the rotor
    // sim state is reported in rotor rotations.
    final double hoodRotorRatio = HOOD_ROTOR_TO_ENCODER * HOOD_ENCODER_TO_MECHANISM;
    hoodSimState.setRawRotorPosition(hoodSimulation.getAngularPosition().times(hoodRotorRatio));
    hoodSimState.setRotorVelocity(hoodSimulation.getAngularVelocity().times(hoodRotorRatio));
    hoodSimState.setRotorAcceleration(
        hoodSimulation.getAngularAcceleration().times(hoodRotorRatio));

    // Defer to `super` to actually update inputs.
    super.updateInputs(inputs);
  }
}
