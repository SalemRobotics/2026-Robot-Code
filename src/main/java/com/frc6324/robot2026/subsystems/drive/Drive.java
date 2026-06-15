package com.frc6324.robot2026.subsystems.drive;

import static com.frc6324.robot2026.generated.TunerConstants.BackLeft;
import static com.frc6324.robot2026.generated.TunerConstants.BackRight;
import static com.frc6324.robot2026.generated.TunerConstants.FrontLeft;
import static com.frc6324.robot2026.generated.TunerConstants.FrontRight;
import static com.frc6324.robot2026.subsystems.drive.DriveConstants.*;

import com.frc6324.robot2026.RobotState;
import com.frc6324.robot2026.subsystems.drive.can.*;
import com.frc6324.robot2026.subsystems.drive.gyro.*;
import com.frc6324.robot2026.subsystems.drive.module.*;
import com.frc6324.robot2026.subsystems.drive.odometry.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public static final Lock ODOMETRY_LOCK = new ReentrantLock(true);

  private final CANBusIO canBus;
  private final CANBusInputsAutoLogged canBusInputs = new CANBusInputsAutoLogged();

  private final GyroIO gyro;
  private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
  private final Alert gyroDisconnectedAlert =
      new Alert(
          "Drivetrain",
          "Drivetrain Gyro is disconnected, using kinematics instead.",
          AlertType.kError);

  private final OdometryThreadIO odometryThread;
  private final OdometryThreadInputsAutoLogged odometryThreadInputs =
      new OdometryThreadInputsAutoLogged();

  private final Module[] modules = new Module[4];
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
  private final RobotState robotState = RobotState.getInstance();

  public final DrivePID autoPID = new DrivePID("Auto", 5, 0, 0, 5, 0, 0);

  public final SysIdRoutine sysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, null, null, state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
          new SysIdRoutine.Mechanism(this::runDriveCharacterization, null, this));

  public Drive(
      ModuleIO fl,
      ModuleIO fr,
      ModuleIO bl,
      ModuleIO br,
      CANBusIO canBus,
      GyroIO gyro,
      OdometryThreadIO odometryThread) {
    this.canBus = canBus;
    this.gyro = gyro;
    this.odometryThread = odometryThread;

    modules[0] = new Module(fl, 0, FrontLeft);
    modules[1] = new Module(fr, 1, FrontRight);
    modules[2] = new Module(bl, 2, BackLeft);
    modules[3] = new Module(br, 3, BackRight);

    odometryThread.start();
  }

  @AutoLogOutput(key = "Odometry/MeasuredModuleStates")
  public SwerveModuleState[] getMeasuredStates() {
    final SwerveModuleState[] states = new SwerveModuleState[4];
    forEachModule((module, idx) -> states[idx] = module.getState());
    return states;
  }

  public void forEachModule(Consumer<Module> consumer) {
    for (final Module module : modules) {
      consumer.accept(module);
    }
  }

  public void forEachModule(ModuleConsumer consumer) {
    for (int i = 0; i < modules.length; i++) {
      consumer.accept(modules[i], i);
    }
  }

  @Override
  public void periodic() {
    ODOMETRY_LOCK.lock();
    try {
      canBus.updateInputs(canBusInputs);
      gyro.updateInputs(gyroInputs);
      odometryThread.updateInputs(odometryThreadInputs);

      Logger.processInputs("Drive/CANBus", canBusInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      Logger.processInputs("Drive/OdometryThread", odometryThreadInputs);

      forEachModule(Module::periodic);
    } finally {
      ODOMETRY_LOCK.unlock();
    }

    if (DriverStation.isDisabled()) {
      stop();

      Logger.recordOutput("Swerve/Setpoints/Raw", new SwerveModuleState[0]);
      Logger.recordOutput("Swerve/Setpoints/Optimized", new SwerveModuleState[0]);
    }

    gyroDisconnectedAlert.set(!gyroInputs.connected);

    final double[] timestamps = odometryThreadInputs.timestamps;
    int sampleCount = timestamps.length;

    for (int sampleIdx = 0; sampleIdx < sampleCount; sampleIdx++) {
      final Optional<Rotation2d> gyroAngle =
          gyroInputs.connected ? Optional.of(gyroInputs.yaw) : Optional.empty();

      final SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
      for (int moduleIdx = 0; moduleIdx < modules.length; moduleIdx++) {
        modulePositions[moduleIdx] = modules[moduleIdx].getOdometryPositions()[sampleIdx];
      }

      robotState.addOdometryObservation(timestamps[sampleIdx], modulePositions, gyroAngle);
    }

    robotState.addStateObservation(getMeasuredStates());
  }

  public void stop() {
    forEachModule(Module::stop);
  }

  public void runDriveCharacterization(Voltage voltage) {
    forEachModule(m -> m.runCharacterization(voltage));
  }

  public void runFieldRelative(ChassisSpeeds speeds) {
    final ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotState.getRotation());

    runRobotRelative(robotRelativeSpeeds);
  }

  public void runRobotRelative(ChassisSpeeds speeds) {
    final ChassisSpeeds discretized = ChassisSpeeds.discretize(speeds, 0.02);
    final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(discretized);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_LINEAR_SPEED);

    Logger.recordOutput("Swerve/Speeds/Commanded", speeds);
    Logger.recordOutput("Swerve/Speeds/Discretized", discretized);
    Logger.recordOutput("Swerve/Setpoints/Raw", moduleStates);

    forEachModule((module, idx) -> module.runSetpoint(moduleStates[idx]));

    Logger.recordOutput("Swerve/Setpoints/Optimized", moduleStates);
  }

  public interface ModuleConsumer {
    public void accept(Module module, int index);
  }
}
