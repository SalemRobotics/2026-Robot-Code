package com.frc6324.robot2026.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface IntakeIO {
  default void setPosition(Angle position) {}

  default void spring() {}

  /**
   * Updates a set of loggable inputs for the intake.
   *
   * @param inputs the inputs to modify.
   */
  void updateInputs(IntakeInputs inputs);

  @AutoLog
  class IntakeInputs {
    public boolean motorConnected = false;

    public Angle motorPosition = Rotations.zero();
    public AngularVelocity motorVelocity = RotationsPerSecond.zero();
    public Voltage motorVoltage = Volts.zero();
    public Current motorStatorCurrent = Amps.zero();
  }
}
