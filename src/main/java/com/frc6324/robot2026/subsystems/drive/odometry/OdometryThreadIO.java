package com.frc6324.robot2026.subsystems.drive.odometry;

import com.frc6324.lib.io.IOLayer;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface OdometryThreadIO extends IOLayer<OdometryThreadIO.OdometryThreadInputs> {
  @AutoLog
  public class OdometryThreadInputs {
    public double[] timestamps = new double[0];
  }

  default void start() {}
}
