package com.frc6324.robot2026.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.frc6324.lib.util.logging.IOLayer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO extends IOLayer<GyroIO.GyroInputs> {
  @AutoLog
  class GyroInputs {
    public boolean connected = false;
    public Rotation2d[] odometryYawPositions = new Rotation2d[0];

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
