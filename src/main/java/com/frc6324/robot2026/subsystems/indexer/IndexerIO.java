package com.frc6324.robot2026.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface IndexerIO {
  default void setKickerVelocity(AngularVelocity velocity) {}

  default void setSpinnerVelocity(AngularVelocity velocity) {}

  default void stopKicker() {}

  default void stopSpinner() {}

  void updateInputs(IndexerInputs inputs);

  @AutoLog
  public class IndexerInputs {
    public boolean spinnerMotorConnected = false;
    public boolean kickerMotorConnected = false;

    public AngularVelocity spinnerVelocity = RadiansPerSecond.zero();
    public Voltage spinnerMotorVoltage = Volts.zero();
    public Current spinnerStatorCurrent = Amps.zero();

    public AngularVelocity kickerVelocity = RadiansPerSecond.zero();
    public Voltage kickerMotorVoltage = Volts.zero();
    public Current kickerStatorCurrent = Amps.zero();
  }
}
