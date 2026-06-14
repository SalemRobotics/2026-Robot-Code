package com.frc6324.robot2026.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.*;

import lombok.RequiredArgsConstructor;
import org.ironmaple.simulation.drivesims.GyroSimulation;

@RequiredArgsConstructor
public class GyroIOSim implements GyroIO {
  private final GyroSimulation simulation;

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.connected = true;

    inputs.yaw = simulation.getGyroReading();
    inputs.yawVelocity = simulation.getMeasuredAngularVelocity();

    inputs.pitch = Degrees.zero();
    inputs.roll = Degrees.zero();

    inputs.accelerationX = MetersPerSecondPerSecond.zero();
    inputs.accelerationY = MetersPerSecondPerSecond.zero();

    inputs.odometryYawPositions = simulation.getCachedGyroReadings();
  }
}
