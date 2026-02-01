package com.frc6324.robot2026.subsystems.rollers;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface RollerIO {
  default void coast() {}

  default void start() {}

  default void stop() {}

  void updateInputs(RollerInputs inputs);

  @AutoLog
  class RollerInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;

    public AngularVelocity leaderVelocity = RotationsPerSecond.zero();
    public AngularAcceleration leaderAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage leaderMotorVoltage = Volts.zero();
    public Current leaderStatorCurrent = Amps.zero();
    public Current leaderTorqueCurrent = Amps.zero();

    public AngularVelocity followerVelocity = RotationsPerSecond.zero();
    public AngularAcceleration followerAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage followerMotorVoltage = Volts.zero();
    public Current followerStatorCurrent = Amps.zero();
    public Current followerTorqueCurrent = Amps.zero();
  }
}
