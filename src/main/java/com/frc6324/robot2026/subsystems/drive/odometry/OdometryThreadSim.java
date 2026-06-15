package com.frc6324.robot2026.subsystems.drive.odometry;

import edu.wpi.first.wpilibj.Timer;

public class OdometryThreadSim implements OdometryThreadIO {
  @Override
  public void updateInputs(OdometryThreadInputs inputs) {
    inputs.timestamps = new double[] {Timer.getTimestamp()};
  }
}
