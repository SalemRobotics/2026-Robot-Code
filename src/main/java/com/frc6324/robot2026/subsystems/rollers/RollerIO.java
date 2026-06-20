package com.frc6324.robot2026.subsystems.rollers;

import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.io.IOLayer;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface RollerIO extends IOLayer<RollerIO.RollerInputs> {
  default void start() {}

  default void stop() {}

  default void outtake() {}

  @AutoLog
  class RollerInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;

    public AngularVelocity leaderVelocity = RotationsPerSecond.zero();
    public Voltage leaderMotorVoltage = Volts.zero();
    public Current leaderStatorCurrent = Amps.zero();

    public AngularVelocity followerVelocity = RotationsPerSecond.zero();
    public Voltage followerMotorVoltage = Volts.zero();
    public Current followerStatorCurrent = Amps.zero();
  }
}
