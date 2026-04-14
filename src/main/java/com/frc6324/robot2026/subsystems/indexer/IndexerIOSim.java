package com.frc6324.robot2026.subsystems.indexer;

import static com.frc6324.robot2026.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.frc6324.lib.util.DeltaTimeCalculator;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim extends IndexerIOTalonFX {
  private final TalonFXSimState kickerSimState = kicker.getSimState();
  private final FlywheelSim kickerSimulation =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              INDEXER_KICKER_GEARBOX, INDEXER_KICKER_MOI, INDEXER_KICKER_REDUCTION),
          INDEXER_KICKER_GEARBOX);

  private final TalonFXSimState spinnerSimState = belt.getSimState();
  private final FlywheelSim spinnerSimulation =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              INDEXER_BELT_GEARBOX, INDEXER_BELT_MOI, INDEXER_BELT_REDUCTION),
          INDEXER_BELT_GEARBOX);

  private final DeltaTimeCalculator deltaTime = new DeltaTimeCalculator();

  public IndexerIOSim() {
    kickerSimState.setMotorType(INDEXER_KICKER_MOTOR_TYPE);
    spinnerSimState.setMotorType(INDEXER_BELT_MOTOR_TYPE);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    final double dt = deltaTime.get();
    final double batteryVoltage = RobotController.getBatteryVoltage();

    kickerSimState.setSupplyVoltage(batteryVoltage);
    spinnerSimState.setSupplyVoltage(batteryVoltage);

    kickerSimulation.setInputVoltage(kickerSimState.getMotorVoltage());
    kickerSimulation.update(dt);

    spinnerSimulation.setInputVoltage(spinnerSimState.getMotorVoltage());
    spinnerSimulation.update(dt);

    kickerSimState.setRotorVelocity(kickerSimulation.getAngularVelocity());
    kickerSimState.setRotorAcceleration(kickerSimulation.getAngularAcceleration());

    spinnerSimState.setRotorVelocity(spinnerSimulation.getAngularVelocity());
    spinnerSimState.setRotorAcceleration(spinnerSimulation.getAngularAcceleration());

    super.updateInputs(inputs);
  }
}
