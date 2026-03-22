package com.frc6324.robot2026.subsystems.rollers;

import static com.frc6324.robot2026.subsystems.rollers.RollerConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.frc6324.lib.util.DeltaTimeCalculator;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public final class RollerIOSim extends RollerIOTalonFX {
  private final TalonFXSimState leaderSimState = leader.getSimState();
  private final TalonFXSimState followerSimState = follower.getSimState();

  private final FlywheelSim leaderSimulation =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(ROLLER_GEARBOX, ROLLER_MOI, ROLLER_REDUCTION),
          ROLLER_GEARBOX);
  private final FlywheelSim followerSimulation =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(ROLLER_GEARBOX, ROLLER_MOI, ROLLER_REDUCTION),
          ROLLER_GEARBOX);

  private final DeltaTimeCalculator deltaTime = new DeltaTimeCalculator();

  public RollerIOSim() {
    leaderSimState.setMotorType(ROLLER_MOTOR_TYPE);
    followerSimState.setMotorType(ROLLER_MOTOR_TYPE);
  }

  @Override
  public void updateInputs(RollerInputs inputs) {
    final double dt = deltaTime.get();
    final double batteryVoltage = RobotController.getBatteryVoltage();

    // Update the CTRE sims' supply voltage measure
    leaderSimState.setSupplyVoltage(batteryVoltage);
    followerSimState.setSupplyVoltage(batteryVoltage);

    // Set the wpilib simulations' input voltage per motor
    leaderSimulation.setInputVoltage(leaderSimState.getMotorVoltage());
    followerSimulation.setInputVoltage(followerSimState.getMotorVoltage());

    // Update the simulations
    leaderSimulation.update(dt);
    followerSimulation.update(dt);

    // Update the sim states of the leader and follower motors
    leaderSimState.setRotorVelocity(leaderSimulation.getAngularVelocity());
    leaderSimState.setRotorAcceleration(leaderSimulation.getAngularAcceleration());
    followerSimState.setRotorVelocity(followerSimulation.getAngularVelocity());
    followerSimState.setRotorAcceleration(followerSimulation.getAngularAcceleration());

    // Defer to `super` to actually update the inputs
    super.updateInputs(inputs);
  }
}
