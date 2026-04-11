// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.frc6324.robot2026.subsystems.drive;

import static com.frc6324.robot2026.subsystems.drive.DrivetrainConstants.MODULE_NAMES;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  private final double wheelRadius;
  private final String logKey;

  private final Alert driveDisconnectedAlert;
  private final Alert steerDisconnectedAlert;
  private final Alert steerEncoderDisconnectedAlert;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public SwerveModule(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.logKey = "Drive/" + MODULE_NAMES[index];

    this.wheelRadius = constants.WheelRadius;
    driveDisconnectedAlert =
        new Alert(MODULE_NAMES[index] + " drive motor is disconnected.", AlertType.kError);
    steerDisconnectedAlert =
        new Alert(MODULE_NAMES[index] + " steer motor is disconnected.", AlertType.kError);
    steerEncoderDisconnectedAlert =
        new Alert(MODULE_NAMES[index] + " steer encoder is disconnected.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);

    final int sampleCount = inputs.odometryTimestamps.length;
    odometryPositions = new SwerveModulePosition[sampleCount];

    for (int i = 0; i < sampleCount; i++) {
      final double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadius;
      final Rotation2d facing = inputs.odometrySteerFacings[i];

      odometryPositions[i] = new SwerveModulePosition(positionMeters, facing);
    }

    // Update disconnection alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    steerDisconnectedAlert.set(!inputs.steerConnected);
    steerEncoderDisconnectedAlert.set(!inputs.steerEncoderConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.steerPosition);

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / wheelRadius);
    io.setSteerPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setSteerPosition(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.stopDriveMotor();
    io.stopSteerMotor();
  }

  /** Returns the current facing of the module. */
  public Rotation2d getAngle() {
    return inputs.steerPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
