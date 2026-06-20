package com.frc6324.robot2026.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.io.IOLayer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface GyroIO extends IOLayer<GyroIO.GyroInputs> {
  @AutoLog
  class GyroInputs {
    public boolean connected = false;
    public Rotation2d[] odometryYawPositions = new Rotation2d[0];

    public Rotation3d rotation3d = Rotation3d.kZero;

    // Yaw (robot rotation)
    public Rotation2d yaw = Rotation2d.kZero;
    public AngularVelocity yawVelocity = DegreesPerSecond.zero();

    // Pitch & roll of the robot for tilt detection
    public Angle pitch = Degrees.zero();
    public Angle roll = Degrees.zero();

    // Acceleration
    public LinearAcceleration accelerationX = MetersPerSecondPerSecond.zero();
    public LinearAcceleration accelerationY = MetersPerSecondPerSecond.zero();
  }
}
