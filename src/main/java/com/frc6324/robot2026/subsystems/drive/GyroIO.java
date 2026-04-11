package com.frc6324.robot2026.subsystems.drive;

import com.frc6324.lib.util.IOLayer;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO extends IOLayer<GyroIO.GyroIOInputs> {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;

    public Rotation2d yawPosition = Rotation2d.kZero;
    public double pitchDegrees = 0;
    public double rollDegrees = 0;

    public double linearAccelerationX = 0;
    public double linearAccelerationY = 0;
    public double linearAccelerationZ = 0;

    public double yawVelocityDPS = 0;
    public double[] odometryYawTimestamps = new double[0];
    public Rotation2d[] odometryYawPositions = new Rotation2d[0];
  }
}
