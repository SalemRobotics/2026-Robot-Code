package com.frc6324.robot2026.subsystems.intake;

import static com.frc6324.robot2026.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.frc6324.lib.util.DeltaTimeCalculator;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public final class IntakeIOSim extends IntakeIOTalonFX {
  private final TalonFXSimState deploySimState = deployTalon.getSimState();
  private final DCMotorSim deploySimulation = new DCMotorSim(LinearSystemId.createDCMotorSystem(INTAKE_DEPLOY_GEARBOX, INTAKE_DEPLOY_MOI, INTAKE_DEPLOY_REDUCTION), INTAKE_DEPLOY_GEARBOX);

  private final TalonFXSimState rollerLeaderSimState = rollerLeaderTalon.getSimState();
  private final TalonFXSimState rollerFollowerSimState = rollerFollowerTalon.getSimState();
  private final FlywheelSim rollerLeaderSimulation = new FlywheelSim(LinearSystemId.createFlywheelSystem(INTAKE_ROLLER_GEARBOX, INTAKE_ROLLER_MOI, INTAKE_ROLLER_REDUCTION), INTAKE_ROLLER_GEARBOX);
  private final FlywheelSim rollerFollowerSimulation = new FlywheelSim(LinearSystemId.createFlywheelSystem(INTAKE_ROLLER_GEARBOX, INTAKE_ROLLER_MOI, INTAKE_ROLLER_REDUCTION), INTAKE_ROLLER_GEARBOX);

  private final DeltaTimeCalculator deltaTime = new DeltaTimeCalculator();

  /**
   * Creates a new sim implementation of the intake.
   */
  public IntakeIOSim() {
    deploySimState.setMotorType(INTAKE_DEPLOY_MOTOR_TYPE);

    rollerLeaderSimState.setMotorType(INTAKE_ROLLER_MOTOR_TYPE);
    rollerFollowerSimState.setMotorType(INTAKE_ROLLER_MOTOR_TYPE);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    // Get the time since the last loop and the robot's current battery
    final double dt = deltaTime.get();
    final double batteryVoltage = RobotController.getBatteryVoltage();

    // Set the supply voltages for all of the sim states
    deploySimState.setSupplyVoltage(batteryVoltage);
    rollerLeaderSimState.setSupplyVoltage(batteryVoltage);
    rollerFollowerSimState.setSupplyVoltage(batteryVoltage);

    // Update the deploy motor sim
    deploySimulation.setInputVoltage(deploySimState.getMotorVoltage());
    deploySimulation.update(dt);

    // Set the phoenix sim state for the deploy motor
    deploySimState.setRawRotorPosition(deploySimulation.getAngularPosition());
    deploySimState.setRotorVelocity(deploySimulation.getAngularVelocity());
    deploySimState.setRotorAcceleration(deploySimulation.getAngularAcceleration());

    // Update the roller simulations
    rollerLeaderSimulation.setInputVoltage(rollerLeaderSimState.getMotorVoltage());
    rollerFollowerSimulation.setInputVoltage(rollerFollowerSimState.getMotorVoltage());
    rollerLeaderSimulation.update(dt);
    rollerFollowerSimulation.update(dt);

    // Update the rollers' sim state
    rollerLeaderSimState.setRotorVelocity(rollerLeaderSimulation.getAngularVelocity());
    rollerLeaderSimState.setRotorAcceleration(rollerLeaderSimulation.getAngularAcceleration());
    rollerFollowerSimState.setRotorVelocity(rollerFollowerSimulation.getAngularVelocity());
    rollerFollowerSimState.setRotorAcceleration(rollerFollowerSimulation.getAngularAcceleration());

    // Defer to `super` to actually update inputs.
    super.updateInputs(inputs);
  }
}
