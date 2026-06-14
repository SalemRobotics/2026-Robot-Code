package com.frc6324.robot2026.subsystems.drive.module;

import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.util.logging.IOLayer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO extends IOLayer<ModuleIO.ModuleInputs> {
  @AutoLog
  public class ModuleInputs {
    public boolean driveMotorConnected = false;
    public boolean steerMotorConnected = false;
    public boolean steerEncoderConnected = false;

    public Angle drivePosition = Rotations.zero();
    public AngularVelocity driveVelocity = RotationsPerSecond.zero();
    public Voltage driveMotorVoltage = Volts.zero();
    public Current driveStatorCurrent = Amps.zero();
    public Current driveTorqueCurrent = Amps.zero();

    public Angle steerAbsolutePosition = Rotations.zero();
    public Rotation2d facing = Rotation2d.kZero;
    public AngularVelocity steerVelocity = RotationsPerSecond.zero();
    public Voltage steerMotorVoltage = Volts.zero();
    public Current steerStatorCurrent = Amps.zero();
    public Current steerTorqueCurrent = Amps.zero();

    public Rotation2d[] odometryAzimuthPositions = new Rotation2d[0];
    public double[] odometryWheelPositions = new double[0];
  }

  void setDriveCurrent(Current current);

  void setDriveVoltage(Voltage volts);

  void setSteerCurrent(Current current);

  void setSteerVoltage(Voltage volts);

  void setDriveVelocity(AngularVelocity velocity);

  void setDriveVelocity(
      AngularVelocity velocity, AngularAcceleration acceleration, Torque feedforward);

  void setSteerPosition(Rotation2d position);

  default void stopDriveMotor() {
    setDriveVelocity(RotationsPerSecond.zero());
  }
}
