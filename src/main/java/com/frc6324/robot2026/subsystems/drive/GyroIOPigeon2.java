package com.frc6324.robot2026.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.frc6324.robot2026.generated.TunerConstants;
import com.frc6324.robot2026.subsystems.drive.PhoenixOdometryThread.GyroOdometry;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final GyroOdometry odometry = PhoenixOdometryThread.getInstance().registerGyro(pigeon);

  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
  private final StatusSignal<Angle> pitch = pigeon.getPitch();
  private final StatusSignal<Angle> roll = pigeon.getRoll();

  private final StatusSignal<LinearAcceleration> accelX = pigeon.getAccelerationX();
  private final StatusSignal<LinearAcceleration> accelY = pigeon.getAccelerationY();
  private final StatusSignal<LinearAcceleration> accelZ = pigeon.getAccelerationZ();

  public GyroIOPigeon2() {
    if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    BaseStatusSignal.setUpdateFrequencyForAll(100, pitch, roll, accelX, accelY, accelZ);
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(pitch, roll, accelX, accelY, accelZ).isOK();

    inputs.yawPosition = pigeon.getRotation2d();
    inputs.yawVelocityDPS = yawVelocity.getValueAsDouble();

    inputs.pitchDegrees = pitch.getValueAsDouble();
    inputs.rollDegrees = roll.getValueAsDouble();

    inputs.linearAccelerationX = accelX.getValueAsDouble();
    inputs.linearAccelerationY = accelY.getValueAsDouble();
    inputs.linearAccelerationZ = accelZ.getValueAsDouble();

    inputs.odometryYawTimestamps =
        odometry.timestamps.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.odometryYawPositions = odometry.yawPositions.toArray(Rotation2d[]::new);
    odometry.clear();
  }
}
