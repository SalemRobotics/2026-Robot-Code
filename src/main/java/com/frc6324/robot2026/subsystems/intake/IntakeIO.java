package com.frc6324.robot2026.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.io.IOLayer;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface IntakeIO extends IOLayer<IntakeIO.IntakeInputs> {
  default void setPosition(Angle position, int slot) {}

  default void setPositionProfiled(Angle position, int slot) {}

  default void spring() {}

  @AutoLog
  class IntakeInputs {
    public boolean motorConnected = false;

    public Angle motorPosition = Rotations.zero();
    public AngularVelocity motorVelocity = RotationsPerSecond.zero();
    public Voltage motorVoltage = Volts.zero();
    public Current motorStatorCurrent = Amps.zero();
  }
}
