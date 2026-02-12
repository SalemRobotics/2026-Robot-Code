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
    public boolean flywheelLeaderConnected = false;
    public boolean flywheelFollowerConnected = false;

    public Angle hoodPosition = Rotations.zero();
    public AngularVelocity hoodVelocity = RotationsPerSecond.zero();
    public double hoodPIDSetpoint = 0;
    public double hoodPIDOutput = 0;
    public Voltage hoodMotorVoltage = Volts.zero();
    public Current hoodStatorCurrent = Amps.zero();
    public Current hoodTorqueCurrent = Amps.zero();

    public AngularVelocity flywheelLeaderVelocity = RotationsPerSecond.zero();
    public AngularAcceleration flywheelLeaderAcceleration = RotationsPerSecondPerSecond.zero();
    public double flywheelLeaderPIDSetpoint = 0;
    public double flywheelLeaderPIDOutput = 0;
    public Voltage flywheelLeaderMotorVoltage = Volts.zero();
    public Current flywheelLeaderStatorCurrent = Amps.zero();
    public Current flywheelLeaderTorqueCurrent = Amps.zero();

    public AngularVelocity flywheelFollowerVelocity = RotationsPerSecond.zero();
    public AngularAcceleration flywheelFollowerAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage flywheelFollowerMotorVoltage = Volts.zero();
    public Current flywheelFollowerStatorCurrent = Amps.zero();
    public Current flywheelFollowerTorqueCurrent = Amps.zero();
  }
}
