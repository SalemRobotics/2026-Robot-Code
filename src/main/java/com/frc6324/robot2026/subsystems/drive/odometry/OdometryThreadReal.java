package com.frc6324.robot2026.subsystems.drive.odometry;

import static com.frc6324.robot2026.subsystems.drive.Drive.ODOMETRY_LOCK;
import static com.frc6324.robot2026.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.traits.HasTalonSignals;
import com.frc6324.lib.util.CommonUtils;
import com.frc6324.lib.util.PhoenixUtil;
import com.frc6324.robot2026.subsystems.drive.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

public class OdometryThreadReal implements OdometryThreadIO {
  private final List<Odometry> items = new ArrayList<>(5);
  private final List<BaseStatusSignal> signals = new ArrayList<>();
  private final BlockingDeque<Double> timestamps = new LinkedBlockingDeque<>(25);
  private final Notifier notifier = new Notifier(this::update);

  public OdometryThreadReal() {
    notifier.setName("Drivetrain Odometry Thread");
  }

  private void addSignals(BaseStatusSignal... newSignals) {
    Collections.addAll(signals, newSignals);
  }

  public Gyro gyro(Pigeon2 pigeon) {
    return new Gyro(pigeon);
  }

  public Module module(HasTalonSignals driveMotor, HasTalonSignals steerMotor) {
    return new Module(driveMotor, steerMotor);
  }

  public void start() {
    BaseStatusSignal.waitForAll(SYNCHRONIZATION_WAIT_PERIOD, signals);
    notifier.startPeriodic(DriveConstants.ODOMETRY_FREQUENCY);
  }

  private void update() {

    try {
      ODOMETRY_LOCK.lock();
      BaseStatusSignal.waitForAll(ODOMETRY_WAIT_PERIOD, signals);
      items.forEach(Odometry::updateOdometry);

      if (timestamps.remainingCapacity() == 0) {
        timestamps.removeFirst();
      }

      timestamps.addLast(PhoenixUtil.averageTimestamp(signals));
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }

  @Override
  public void updateInputs(OdometryThreadInputs inputs) {
    inputs.timestamps = CommonUtils.mapToDouble(d -> d, timestamps);
    timestamps.clear();
  }

  private interface Odometry {
    void updateOdometry();
  }

  public class Gyro implements Odometry {
    private final BlockingDeque<Rotation2d> yawPositions = new LinkedBlockingDeque<>(25);

    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;

    public Gyro(Pigeon2 pigeon) {
      yaw = pigeon.getYaw();
      yawVelocity = pigeon.getAngularVelocityZWorld();

      items.add(this);
      addSignals(yaw, yawVelocity);
    }

    public Rotation2d[] getYawPositions() {
      final Rotation2d[] positions = yawPositions.toArray(Rotation2d[]::new);
      yawPositions.clear();
      return positions;
    }

    @Override
    public void updateOdometry() {
      if (yawPositions.remainingCapacity() == 0) {
        yawPositions.removeFirst();
      }

      final Angle yawAngle = BaseStatusSignal.getLatencyCompensatedValue(yaw, yawVelocity);
      yawPositions.addLast(new Rotation2d(yawAngle));
    }
  }

  public class Module implements Odometry {
    private final BlockingDeque<Rotation2d> azimuthPositions = new LinkedBlockingDeque<>(25);
    private final BlockingDeque<Angle> wheelPositions = new LinkedBlockingDeque<>(25);

    private final StatusSignal<Angle> azimuthPosition;
    private final StatusSignal<AngularVelocity> azimuthVelocity;

    private final StatusSignal<Angle> wheelPosition;
    private final StatusSignal<AngularVelocity> wheelVelocity;

    public Module(HasTalonSignals driveMotor, HasTalonSignals steerMotor) {
      azimuthPosition = steerMotor.getPosition();
      azimuthVelocity = steerMotor.getVelocity();

      wheelPosition = driveMotor.getPosition();
      wheelVelocity = driveMotor.getVelocity();

      items.add(this);
      addSignals(wheelPosition, wheelVelocity, azimuthPosition, azimuthVelocity);
    }

    public Rotation2d[] getAzimuthPositions() {
      final Rotation2d[] positions = azimuthPositions.toArray(Rotation2d[]::new);
      azimuthPositions.clear();
      return positions;
    }

    public Angle[] getWheelPositions() {
      final Angle[] positions = wheelPositions.toArray(Angle[]::new);
      wheelPositions.clear();
      return positions;
    }

    @Override
    public void updateOdometry() {
      // Only check if one queue has no more space because they always have the same number of items
      if (azimuthPositions.remainingCapacity() == 0) {
        azimuthPositions.removeFirst();
        wheelPositions.removeFirst();
      }

      final Angle azimuthAngle =
          BaseStatusSignal.getLatencyCompensatedValue(azimuthPosition, azimuthVelocity);
      final Angle wheelAngle =
          BaseStatusSignal.getLatencyCompensatedValue(wheelPosition, wheelVelocity);

      azimuthPositions.add(new Rotation2d(azimuthAngle));
      wheelPositions.add(wheelAngle);
    }
  }
}
