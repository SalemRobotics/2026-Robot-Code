package com.frc6324.robot2026.subsystems.intake;

import static com.frc6324.robot2026.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.frc6324.lib.util.DeltaTimeCalculator;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public final class IntakeIOSim extends IntakeIOTalonFX {
  private final TalonFXSimState deploySimState = talon.getSimState();
  private final DCMotorSim deploySimulation =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(INTAKE_GEARBOX, INTAKE_MOI, INTAKE_REDUCTION),
          INTAKE_GEARBOX);

  private final DeltaTimeCalculator deltaTime = new DeltaTimeCalculator();

  /** Creates a new sim implementation of the intake. */
  public IntakeIOSim() {
    deploySimState.setMotorType(INTAKE_MOTOR_TYPE);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    // Get the time since the last loop and the robot's current battery
    final double dt = deltaTime.get();
    final double batteryVoltage = RobotController.getBatteryVoltage();

    // Set the supply voltages for all of the sim states
    deploySimState.setSupplyVoltage(batteryVoltage);

    // Update the deploy motor sim
    deploySimulation.setInputVoltage(deploySimState.getMotorVoltage());
    deploySimulation.update(dt);

    // Set the phoenix sim state for the deploy motor
    deploySimState.setRawRotorPosition(deploySimulation.getAngularPosition());
    deploySimState.setRotorVelocity(deploySimulation.getAngularVelocity());
    deploySimState.setRotorAcceleration(deploySimulation.getAngularAcceleration());

    // Defer to `super` to actually update inputs.
    super.updateInputs(inputs);
  }
}
