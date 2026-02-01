package com.frc6324.robot2026.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface ClimberIO {
  void updateInputs(ClimberInputs inputs);

  /** Deploys the climber. */
  default void deploy() {}

  /** Stops the climber motor. */
  default void stop() {}

  /** Stows the climber, which can also raise the robot. */
  default void stow() {}

  /** Inputs for climber IO implementations. */
  @AutoLog
  class ClimberInputs {
    public boolean motorConnected = false;
    public AngularVelocity motorVelocity = RadiansPerSecond.zero();
    public Voltage motorVoltage = Volts.zero();
    public Current statorCurrent = Amps.zero();
    public Current supplyCurrent = Amps.zero();
    public Current torqueCurrent = Amps.zero();
  }
}
