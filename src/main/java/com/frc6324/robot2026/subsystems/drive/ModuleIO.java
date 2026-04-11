// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.frc6324.robot2026.subsystems.drive;

import com.frc6324.lib.util.IOLayer;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO extends IOLayer<ModuleIO.ModuleIOInputs> {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean steerConnected = false;
    public boolean steerEncoderConnected = false;
    public Rotation2d steerAbsolutePosition = Rotation2d.kZero;
    public Rotation2d steerPosition = Rotation2d.kZero;
    public double steerVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometrySteerFacings = new Rotation2d[] {};
  }

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the steer motor at the specified open loop value. */
  public default void setSteerOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the steer motor to the specified rotation. */
  public default void setSteerPosition(Rotation2d rotation) {}

  /** Stops the drive motor. */
  public default void stopDriveMotor() {
    setDriveOpenLoop(0);
  }

  /** Stops the steer motor. */
  public default void stopSteerMotor() {
    setSteerOpenLoop(0);
  }
}
