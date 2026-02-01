package com.frc6324.robot2026.subsystems.climber;

import static com.frc6324.robot2026.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.frc6324.lib.util.DeltaTimeCalculator;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

/** A simulation implementation for the climber's I/O. */
public final class ClimberIOSim extends ClimberIOTalonFX {
  private final DCMotorSim simulation = new DCMotorSim(CLIMBER_LINEAR_SYSTEM, CLIMBER_GEARBOX);
  private final TalonFXSimState talonSim = talon.getSimState();
  private final DeltaTimeCalculator delta = new DeltaTimeCalculator();

  /** Creates an I/O implementation for the climber that simulates a TalonFX motor. */
  public ClimberIOSim() {
    talonSim.setMotorType(CLIMBER_MOTOR_TYPE);
  }

  @Override
  public void updateInputs(ClimberInputs inputs) {
    simulation.setInputVoltage(talonSim.getMotorVoltage());
    simulation.update(delta.get());

    // Log the angular position
    final Angle position = simulation.getAngularPosition();
    Logger.recordOutput("ClimberSim/Position", position);

    // Log the angular velocity
    final AngularVelocity velocity = simulation.getAngularVelocity();
    Logger.recordOutput("ClimberSim/Velocity", velocity);

    // Log the angular acceleration
    final AngularAcceleration acceleration = simulation.getAngularAcceleration();
    Logger.recordOutput("ClimberSim/Acceleration", acceleration);

    // Set the TalonFX's state
    talonSim.setRawRotorPosition(position);
    talonSim.setRotorVelocity(velocity);
    talonSim.setRotorAcceleration(acceleration);

    // Defer to super to update inputs
    super.updateInputs(inputs);
  }
}
