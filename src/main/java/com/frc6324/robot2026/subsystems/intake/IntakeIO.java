package com.frc6324.robot2026.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface IntakeIO {
  default void deploy() {}

  default void runRollers() {}

  default void spring() {}

  default void stopDeploy() {}

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
    public boolean deployMotorConnected = false;
    public boolean rollerLeaderConnected = false;
    public boolean rollerFollowerConnected = false;

    public Angle deployPosition = Rotations.zero();
    public AngularVelocity deployMotorVelocity = RotationsPerSecond.zero();
    public Voltage deployVoltage = Volts.zero();
    public Current deployStatorCurrent = Amps.zero();
    public Current deployTorqueCurrent = Amps.zero();

    public AngularVelocity rollerLeaderVelocity = RotationsPerSecond.zero();
    public AngularAcceleration rollerLeaderAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage rollerLeaderVoltage = Volts.zero();
    public Current rollerLeaderStatorCurrent = Amps.zero();
    public Current rollerLeaderTorqueCurrent = Amps.zero();

    public AngularVelocity rollerFollowerVelocity = RotationsPerSecond.zero();
    public AngularAcceleration rollerFollowerAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage rollerFollowerVoltage = Volts.zero();
    public Current rollerFollowerStatorCurrent = Amps.zero();
    public Current rollerFollowerTorqueCurrent = Amps.zero();
  }
}
