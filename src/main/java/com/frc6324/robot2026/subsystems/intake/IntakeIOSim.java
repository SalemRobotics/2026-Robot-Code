package com.frc6324.robot2026.subsystems.intake;

import static com.frc6324.robot2026.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.frc6324.lib.util.DeltaTimeCalculator;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public final class IntakeIOSim extends IntakeIOTalonFX {
  private final TalonFXSimState extensionSimState = extensionTalon.getSimState();
  private final TalonFXSimState spinLeaderSimState = spinLeaderTalon.getSimState();
  private final TalonFXSimState spinFollowerSimState = spinFollowerTalon.getSimState();

  private final DCMotorSim extensionSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(INTAKE_GEARBOX, INTAKE_MOI, INTAKE_REDUCTION),
          INTAKE_GEARBOX);
  private final DeltaTimeCalculator delta = new DeltaTimeCalculator();

  @Override
  public void updateInputs(IntakeInputs inputs) {
    final double batteryVolts = RobotController.getBatteryVoltage();
    extensionSimState.setSupplyVoltage(batteryVolts);
    spinLeaderSimState.setSupplyVoltage(batteryVolts);
    spinFollowerSimState.setSupplyVoltage(batteryVolts);

    extensionSim.setInputVoltage(extensionSimState.getMotorVoltage());
    extensionSim.update(delta.get());

    extensionSimState.setRawRotorPosition(extensionSim.getAngularPosition());
    extensionSimState.setRotorVelocity(extensionSim.getAngularVelocity());
    extensionSimState.setRotorAcceleration(extensionSim.getAngularAcceleration());

    super.updateInputs(inputs);
  }
}
