package com.frc6324.robot2026.subsystems.drive.odometry;

import static com.frc6324.robot2026.subsystems.drive.DriveConstants.SIMULATION_TICKS_PER_LOOP;

import edu.wpi.first.wpilibj.Timer;

public class OdometryThreadSim implements OdometryThreadIO {
  private static final double PERIOD = 20 / SIMULATION_TICKS_PER_LOOP;

  private double lastTimestamp = 0;

  @Override
  public void start() {
    lastTimestamp = Timer.getTimestamp();
  }

  @Override
  public void updateInputs(OdometryThreadInputs inputs) {
    final double[] timestamps = new double[SIMULATION_TICKS_PER_LOOP];

    for (int i = 0; i < SIMULATION_TICKS_PER_LOOP; i++) {
      timestamps[i] = lastTimestamp + PERIOD * i;
    }

    inputs.timestamps = timestamps;
    lastTimestamp = Timer.getTimestamp();
  }
}
