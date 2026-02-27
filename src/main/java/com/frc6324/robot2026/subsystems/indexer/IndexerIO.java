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
    public AngularAcceleration spinnerAcceleration = RadiansPerSecondPerSecond.zero();
    public Voltage spinnerMotorVoltage = Volts.zero();
    public Current spinnerStatorCurrent = Amps.zero();
    public Current spinnerTorqueCurrent = Amps.zero();
    public AngularVelocity spinnerTargetVelocity = RadiansPerSecond.zero();
    public double spinnerPIDOutput = 0;

    public AngularVelocity kickerVelocity = RadiansPerSecond.zero();
    public AngularAcceleration kickerAcceleration = RadiansPerSecondPerSecond.zero();
    public Voltage kickerMotorVoltage = Volts.zero();
    public Current kickerStatorCurrent = Amps.zero();
    public Current kickerTorqueCurrent = Amps.zero();
    public AngularVelocity kickerTargetVelocity = RadiansPerSecond.zero();
    public double kickerPIDOutput = 0;
  }
}
