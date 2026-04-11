package com.frc6324.robot2026.subsystems.drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.frc6324.robot2026.sim.MapleSimManager;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation simulation =
      MapleSimManager.getInstance().getMainRobotDriveSimulation().getGyroSimulation();

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;

    inputs.yawPosition = simulation.getGyroReading();
    inputs.yawVelocityDPS = simulation.getMeasuredAngularVelocity().in(DegreesPerSecond);

    inputs.rollDegrees = 0;
    inputs.pitchDegrees = 0;

    inputs.linearAccelerationX = 0;
    inputs.linearAccelerationY = 0;
    inputs.linearAccelerationZ = 0;

    inputs.odometryYawPositions = simulation.getCachedGyroReadings();
  }
}
