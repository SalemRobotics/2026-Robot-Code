package com.frc6324.robot2026.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

public class ModuleIOReplay implements ModuleIO {
  @Override
  public void setDriveCurrent(Current current) {}

  @Override
  public void setDriveVoltage(Voltage volts) {}

  @Override
  public void setSteerCurrent(Current current) {}

  @Override
  public void setSteerVoltage(Voltage volts) {}

  @Override
  public void setDriveVelocity(AngularVelocity velocity) {}

  @Override
  public void setDriveVelocity(
      AngularVelocity velocity, AngularAcceleration acceleration, Torque feedforward) {}

  @Override
  public void setSteerPosition(Rotation2d position) {}

  @Override
  public void updateInputs(ModuleInputs inputs) {}
}
