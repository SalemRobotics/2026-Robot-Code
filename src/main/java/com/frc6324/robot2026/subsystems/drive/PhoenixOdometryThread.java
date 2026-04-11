package com.frc6324.robot2026.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.traits.HasTalonSignals;
import com.frc6324.lib.util.PhoenixUtil;
import com.frc6324.robot2026.generated.TunerConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public final class PhoenixOdometryThread {
  private static final boolean IS_CAN_FD = TunerConstants.kCANBus.isNetworkFD();
  public static final Lock ODOMETRY_LOCK = new ReentrantLock(true);

  private static PhoenixOdometryThread instance = null;

  private final StatusSignalCollection signals = new StatusSignalCollection();
  private final List<OdometryCallback> callbacks = new ArrayList<>(5);
  private final Notifier notifier =
      new Notifier(
          () -> {
            try {
              ODOMETRY_LOCK.lock();

              if (IS_CAN_FD) {
                signals.waitForAll(1 / DrivetrainConstants.ODOMETRY_UPDATE_FREQUENCY);
              } else {
                signals.refreshAll();
              }

              callbacks.forEach(OdometryCallback::updateOdometry);
            } finally {
              ODOMETRY_LOCK.unlock();
            }
          });

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    notifier.setName("Drivetrain Odometry Thread");
  }

  public GyroOdometry registerGyro(Pigeon2 pigeon) {
    final GyroOdometry odom = new GyroOdometry(pigeon);
    callbacks.add(odom);

    return odom;
  }

  public ModuleOdometry registerModule(HasTalonSignals driveMotor, HasTalonSignals steerMotor) {
    final ModuleOdometry odom = new ModuleOdometry(driveMotor, steerMotor);
    callbacks.add(odom);

    return odom;
  }

  public void start() {
    notifier.startPeriodic(1.0 / DrivetrainConstants.ODOMETRY_UPDATE_FREQUENCY);
  }

  private interface OdometryCallback {
    void updateOdometry();
  }

  public final class GyroOdometry implements OdometryCallback {
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;

    public final BlockingDeque<Double> timestamps = new LinkedBlockingDeque<>(25);
    public final BlockingDeque<Rotation2d> yawPositions = new LinkedBlockingDeque<>(25);

    GyroOdometry(Pigeon2 pigeon) {
      this.yaw = pigeon.getYaw();
      this.yawVelocity = pigeon.getAngularVelocityZWorld();

      // Add this gyro's signals to the drivetrain's collection
      signals.addSignals(yaw, yawVelocity);
      BaseStatusSignal.setUpdateFrequencyForAll(
          DrivetrainConstants.ODOMETRY_UPDATE_FREQUENCY, yaw, yawVelocity);
    }

    public void clear() {
      timestamps.clear();
      yawPositions.clear();
    }

    @Override
    public void updateOdometry() {
      final double yawDegrees =
          BaseStatusSignal.getLatencyCompensatedValueAsDouble(yaw, yawVelocity);
      final double timestamp = PhoenixUtil.averageTimestamp(yaw, yawVelocity);

      if (timestamps.remainingCapacity() == 0) {
        timestamps.removeFirst();
        yawPositions.removeFirst();
      }

      timestamps.addLast(timestamp);
      yawPositions.addLast(Rotation2d.fromDegrees(yawDegrees));
    }
  }

  public final class ModuleOdometry implements OdometryCallback {
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Angle> steerPosition;
    private final StatusSignal<AngularVelocity> steerVelocity;

    public final BlockingDeque<Double> timestamps = new LinkedBlockingDeque<>(25);
    public final BlockingDeque<Angle> drivePositions = new LinkedBlockingDeque<>(25);
    public final BlockingDeque<Rotation2d> steerPositions = new LinkedBlockingDeque<>(25);

    public ModuleOdometry(HasTalonSignals driveMotor, HasTalonSignals steerMotor) {
      this.drivePosition = driveMotor.getPosition();
      this.driveVelocity = driveMotor.getVelocity();

      this.steerPosition = steerMotor.getPosition();
      this.steerVelocity = steerMotor.getVelocity();

      BaseStatusSignal.setUpdateFrequencyForAll(
          DrivetrainConstants.ODOMETRY_UPDATE_FREQUENCY,
          drivePosition,
          driveVelocity,
          steerPosition,
          steerVelocity);
    }

    public void clear() {
      timestamps.clear();
      drivePositions.clear();
      steerPositions.clear();
    }

    @Override
    public void updateOdometry() {
      final Angle drivePos =
          BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
      final double steerPos =
          BaseStatusSignal.getLatencyCompensatedValueAsDouble(steerPosition, steerVelocity);
      final double timestamp =
          PhoenixUtil.averageTimestamp(drivePosition, driveVelocity, steerPosition, steerVelocity);

      if (timestamps.remainingCapacity() == 0) {
        timestamps.removeFirst();
        drivePositions.removeFirst();
        steerPositions.removeFirst();
      }

      timestamps.addLast(timestamp);
      drivePositions.addLast(drivePos);
      steerPositions.addLast(Rotation2d.fromRotations(steerPos));
    }
  }
}
