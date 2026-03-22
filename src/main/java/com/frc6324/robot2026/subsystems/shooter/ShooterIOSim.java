package com.frc6324.robot2026.subsystems.shooter;

import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.FlywheelConstants.*;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.HoodConstants.*;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.frc6324.lib.util.DeltaTimeCalculator;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim extends ShooterIOTalonFX {
  private final DCMotorSim hoodSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              HOOD_GEARBOX, HOOD_MOI.in(KilogramSquareMeters), HOOD_REDUCTION),
          HOOD_GEARBOX);
  private final FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              FLYWHEEL_GEARBOX, FLYWHEEL_MOI.in(KilogramSquareMeters), 1),
          FLYWHEEL_GEARBOX);

  private final TalonFXSimState hoodSimState = hoodTalon.getSimState();
  private final TalonFXSimState flywheelSimState = flywheelLeader.getSimState();
  private final DeltaTimeCalculator deltaTime = new DeltaTimeCalculator();

  public ShooterIOSim() {
    hoodSimState.setMotorType(HOOD_MOTOR_TYPE);
    flywheelSimState.setMotorType(FLYWHEEL_MOTOR_TYPE);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    final double dt = deltaTime.get();
    final double batteryVoltage = RobotController.getBatteryVoltage();

    // Inform the motors of the supply voltage
    hoodSimState.setSupplyVoltage(batteryVoltage);
    flywheelSimState.setSupplyVoltage(batteryVoltage);

    // Update the sims
    hoodSim.setInputVoltage(hoodSimState.getMotorVoltage());
    hoodSim.update(dt);
    flywheelSim.setInputVoltage(flywheelSimState.getMotorVoltage());
    flywheelSim.update(dt);

    // Update the hood's state
    hoodSimState.setRawRotorPosition(hoodSim.getAngularPosition());
    hoodSimState.setRotorVelocity(hoodSim.getAngularVelocity());
    hoodSimState.setRotorAcceleration(hoodSim.getAngularAcceleration());

    // Update the flywheel's state
    flywheelSimState.setRotorVelocity(flywheelSim.getAngularVelocity());
    flywheelSimState.setRotorAcceleration(flywheelSim.getAngularAcceleration());

    // Defer to `super` to actually update inputs.
    super.updateInputs(inputs);
  }
}
