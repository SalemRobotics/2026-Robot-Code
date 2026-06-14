package com.frc6324.robot2026.subsystems.drive.odometry;

import com.frc6324.lib.util.logging.IOLayer;
import org.littletonrobotics.junction.AutoLog;

public interface OdometryThreadIO extends IOLayer<OdometryThreadIO.OdometryThreadInputs> {
  @AutoLog
  public class OdometryThreadInputs {
    public double[] timestamps = new double[0];
  }

  default void start() {}
}
