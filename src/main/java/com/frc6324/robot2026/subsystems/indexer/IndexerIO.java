package com.frc6324.robot2026.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.io.IOLayer;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface IndexerIO extends IOLayer<IndexerIO.IndexerInputs> {
  default void setKickerVelocity(AngularVelocity velocity) {}

  default void setBeltVelocity(AngularVelocity velocity) {}

  default void stopKicker() {}

  default void stopBelt() {}

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
