package com.frc6324.robot2026.subsystems.drive;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public final class SwerveWidget implements Sendable {
  private final SwerveDrive drive;

  public SwerveWidget(SwerveDrive drive) {
    this.drive = drive;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");

    // Initialize the front left module
    builder.addDoubleProperty(
        "Front Left Angle", () -> drive.getState().ModulePositions[0].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Left Velocity", () -> drive.getState().ModuleStates[0].speedMetersPerSecond, null);

    // Initialize the front right module
    builder.addDoubleProperty(
        "Front Right Angle", () -> drive.getState().ModulePositions[1].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Right Velocity", () -> drive.getState().ModuleStates[1].speedMetersPerSecond, null);

    // Initialize the back left module
    builder.addDoubleProperty(
        "Back Left Angle", () -> drive.getState().ModulePositions[2].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Back Left Velocity", () -> drive.getState().ModuleStates[2].speedMetersPerSecond, null);

    // Initialize the back right module
    builder.addDoubleProperty(
        "Back Right Angle", () -> drive.getState().ModulePositions[3].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Back Right Velocity", () -> drive.getState().ModuleStates[3].speedMetersPerSecond, null);
  }
}
