package com.frc6324.robot2026.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface IntakeIO {
  default void deploy() {}

  default void runRollers() {}

  default void spring() {}

  default void stopRollers() {}

  default void stow() {}

  /**
   * Updates a set of loggable inputs for the intake.
   *
   * @param inputs the inputs to modify.
   */
  void updateInputs(IntakeInputs inputs);

  @AutoLog
  class IntakeInputs {
    public boolean extensionMotorConnected = false;
    public boolean spinLeaderConnected = false;
    public boolean spinFollowerConnected = false;

    public Angle extensionMotorPosition = Rotations.zero();
  }
}
