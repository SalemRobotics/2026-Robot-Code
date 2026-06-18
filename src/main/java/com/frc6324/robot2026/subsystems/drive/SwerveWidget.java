package com.frc6324.robot2026.subsystems.drive;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class SwerveWidget implements Sendable {
  private final Drive drive;

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");

    // Initialize the front left module
    builder.addDoubleProperty(
        "Front Left Angle", () -> drive.getMeasuredPositions()[0].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Left Velocity", () -> drive.getMeasuredStates()[0].speedMetersPerSecond, null);

    // Initialize the front right module
    builder.addDoubleProperty(
        "Front Right Angle", () -> drive.getMeasuredPositions()[1].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Right Velocity", () -> drive.getMeasuredStates()[1].speedMetersPerSecond, null);

    // Initialize the back left module
    builder.addDoubleProperty(
        "Back Left Angle", () -> drive.getMeasuredPositions()[2].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Back Left Velocity", () -> drive.getMeasuredStates()[2].speedMetersPerSecond, null);

    // Initialize the back right module
    builder.addDoubleProperty(
        "Back Right Angle", () -> drive.getMeasuredPositions()[3].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Back Right Velocity", () -> drive.getMeasuredStates()[3].speedMetersPerSecond, null);
  }
}
