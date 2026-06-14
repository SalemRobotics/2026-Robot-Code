package com.frc6324.robot2026.subsystems.drive.gyro;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.frc6324.robot2026.generated.TunerConstants;
import com.frc6324.robot2026.subsystems.drive.odometry.OdometryThreadReal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final OdometryThreadReal.Gyro odometry;

  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  private final StatusSignal<Angle> pitch = pigeon.getPitch();
  private final StatusSignal<Angle> roll = pigeon.getRoll();

  private final StatusSignal<LinearAcceleration> accelerationX = pigeon.getAccelerationX();
  private final StatusSignal<LinearAcceleration> accelerationY = pigeon.getAccelerationY();

  private final StatusSignalCollection signals =
      new StatusSignalCollection(yaw, yawVelocity, pitch, roll, accelerationX, accelerationY);

  public GyroIOPigeon2(OdometryThreadReal odometryThread) {
    odometry = odometryThread.gyro(pigeon);

    tryUntilOk(
        5, () -> pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs));
    tryUntilOk(5, () -> pigeon.setYaw(0));

    BaseStatusSignal.setUpdateFrequencyForAll(50, pitch, roll, accelerationX, accelerationY);
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.connected = signals.refreshAll().isOK();

    inputs.yaw = new Rotation2d(yaw.getValue());
    inputs.yawVelocity = yawVelocity.getValue();

    inputs.pitch = pitch.getValue();
    inputs.roll = roll.getValue();

    inputs.accelerationX = accelerationX.getValue();
    inputs.accelerationY = accelerationY.getValue();

    inputs.odometryYawPositions = odometry.getYawPositions();
  }
}
