package com.frc6324.robot2026.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface ShooterIO {
  default void coastFlywheel() {}

  default void setFlywheelVelocity(AngularVelocity velocity) {}

  default void setHoodAngle(Angle angle) {}

  default void stopHoodMotor() {}

  void updateInputs(ShooterInputs inputs);

  @AutoLog
  public class ShooterInputs {
    public boolean hoodConnected = false;
    public boolean flywheelConnected = false;

    public Angle hoodPosition = Rotations.zero();
    public AngularVelocity hoodVelocity = RotationsPerSecond.zero();
    public double hoodPIDSetpoint = 0;
    public double hoodPIDOutput = 0;
    public Voltage hoodMotorVoltage = Volts.zero();
    public Current hoodStatorCurrent = Amps.zero();
    public Current hoodTorqueCurrent = Amps.zero();

    public AngularVelocity flywheelVelocity = RotationsPerSecond.zero();
    public AngularAcceleration flywheelAcceleration = RotationsPerSecondPerSecond.zero();
    public double flywheelPIDSetpoint = 0;
    public double flywheelPIDOutput = 0;
    public Voltage flywheelMotorVoltage = Volts.zero();
    public Current flywheelStatorCurrent = Amps.zero();
    public Current flywheelTorqueCurrent = Amps.zero();
  }
}
