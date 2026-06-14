package com.frc6324.robot2026.subsystems.drive.module;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.frc6324.lib.util.CommonUtils;
import com.frc6324.robot2026.generated.TunerConstants;
import com.frc6324.robot2026.subsystems.drive.odometry.OdometryThreadReal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

public class ModuleIOTalonFX implements ModuleIO {
  private final SwerveModuleConstants<?, ?, ?> constants;

  private final TalonFX driveTalon;
  private final TalonFX steerTalon;
  private final CANcoder cancoder;

  private final OdometryThreadReal.Module odometry;

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveMotorVoltage;
  private final StatusSignal<Current> driveStatorCurrent;
  private final StatusSignal<Current> driveTorqueCurrent;

  private final StatusSignal<Angle> steerAbsolutePosition;
  private final StatusSignal<Angle> steerPosition;
  private final StatusSignal<AngularVelocity> steerVelocity;
  private final StatusSignal<Voltage> steerMotorVoltage;
  private final StatusSignal<Current> steerStatorCurrent;
  private final StatusSignal<Current> steerTorqueCurrent;

  private final VoltageOut voltage = new VoltageOut(0);
  private final PositionVoltage positionVoltage = new PositionVoltage(0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  private final TorqueCurrentFOC torqueCurrent = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrent = new PositionTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrent = new VelocityTorqueCurrentFOC(0);

  private final DCMotor gearbox;

  public ModuleIOTalonFX(
      OdometryThreadReal odometryThread,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          moduleConstants) {
    constants = moduleConstants;

    driveTalon = new TalonFX(moduleConstants.DriveMotorId, TunerConstants.kCANBus);
    steerTalon = new TalonFX(moduleConstants.SteerMotorId, TunerConstants.kCANBus);
    cancoder = new CANcoder(moduleConstants.EncoderId, TunerConstants.kCANBus);

    final TalonFXConfiguration driveConfig = moduleConstants.DriveMotorInitialConfigs;
    driveConfig.Feedback.SensorToMechanismRatio = moduleConstants.DriveMotorGearRatio;
    driveConfig.MotorOutput.Inverted =
        moduleConstants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = moduleConstants.DriveMotorGains;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = moduleConstants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -moduleConstants.SlipCurrent;

    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig));
    tryUntilOk(5, () -> driveTalon.setPosition(0));

    final TalonFXConfiguration steerConfig = moduleConstants.SteerMotorInitialConfigs;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.Feedback.FeedbackRemoteSensorID = moduleConstants.EncoderId;
    steerConfig.Feedback.FeedbackSensorSource =
        switch (moduleConstants.FeedbackSource) {
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default -> FeedbackSensorSourceValue.RemoteCANcoder;
        };
    steerConfig.Feedback.RotorToSensorRatio = moduleConstants.SteerMotorGearRatio;
    steerConfig.MotionMagic.MotionMagicCruiseVelocity = 100 / moduleConstants.SteerMotorGearRatio;
    steerConfig.MotionMagic.MotionMagicAcceleration =
        steerConfig.MotionMagic.MotionMagicCruiseVelocity * 10;
    steerConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * moduleConstants.SteerMotorGearRatio;
    steerConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted =
        moduleConstants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    steerConfig.Slot0 = moduleConstants.SteerMotorGains;

    tryUntilOk(5, () -> steerTalon.getConfigurator().apply(steerConfig));

    final CANcoderConfiguration cancoderConfig = moduleConstants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = moduleConstants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        moduleConstants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig));

    odometry = odometryThread.module(driveTalon, steerTalon);

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveMotorVoltage = driveTalon.getMotorVoltage();
    driveStatorCurrent = driveTalon.getStatorCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();

    steerAbsolutePosition = cancoder.getAbsolutePosition();
    steerPosition = steerTalon.getPosition();
    steerVelocity = steerTalon.getVelocity();
    steerMotorVoltage = steerTalon.getMotorVoltage();
    steerStatorCurrent = steerTalon.getStatorCurrent();
    steerTorqueCurrent = steerTalon.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        driveMotorVoltage,
        driveStatorCurrent,
        driveTorqueCurrent,
        steerMotorVoltage,
        steerStatorCurrent,
        steerTorqueCurrent,
        steerAbsolutePosition);

    ParentDevice.optimizeBusUtilizationForAll(driveTalon, steerTalon, cancoder);

    final DCMotor driveMotor =
        switch (moduleConstants.DriveMotorClosedLoopOutput) {
          case TorqueCurrentFOC -> DCMotor.getKrakenX60Foc(1);
          case Voltage -> DCMotor.getKrakenX60(1);
        };

    gearbox = driveMotor.withReduction(moduleConstants.DriveMotorGearRatio);
  }

  private Current calculateFeedForwardCurrent(Torque ff) {
    final double current = gearbox.getCurrent(ff.in(NewtonMeters));
    return Amps.of(current);
  }

  private Voltage calculateFeedForwardVoltage(Torque ff) {
    final double velocity = driveVelocity.refresh().getValue().in(RadiansPerSecond);
    final double voltage = gearbox.getVoltage(ff.in(NewtonMeters), velocity);
    return Volts.of(voltage);
  }

  @Override
  public void setDriveCurrent(Current current) {
    driveTalon.setControl(torqueCurrent.withOutput(current));
  }

  @Override
  public void setDriveVoltage(Voltage volts) {
    driveTalon.setControl(voltage.withOutput(volts));
  }

  @Override
  public void setSteerCurrent(Current current) {
    steerTalon.setControl(torqueCurrent.withOutput(current));
  }

  @Override
  public void setSteerVoltage(Voltage volts) {
    steerTalon.setControl(voltage.withOutput(volts));
  }

  @Override
  public void setDriveVelocity(AngularVelocity velocity) {
    switch (constants.DriveMotorClosedLoopOutput) {
      case TorqueCurrentFOC ->
          driveTalon.setControl(
              velocityTorqueCurrent.withVelocity(velocity).withAcceleration(0).withFeedForward(0));
      case Voltage ->
          driveTalon.setControl(
              velocityVoltage.withVelocity(velocity).withAcceleration(0).withFeedForward(0));
    }
  }

  @Override
  public void setDriveVelocity(
      AngularVelocity velocity, AngularAcceleration acceleration, Torque feedforward) {
    switch (constants.DriveMotorClosedLoopOutput) {
      case TorqueCurrentFOC -> {
        final Current current = calculateFeedForwardCurrent(feedforward);

        driveTalon.setControl(
            velocityTorqueCurrent
                .withVelocity(velocity)
                .withAcceleration(acceleration)
                .withFeedForward(current));
      }
      case Voltage -> {
        final Voltage voltage = calculateFeedForwardVoltage(feedforward);

        driveTalon.setControl(
            velocityVoltage
                .withVelocity(velocity)
                .withAcceleration(acceleration)
                .withFeedForward(voltage));
      }
    }
  }

  @Override
  public void setSteerPosition(Rotation2d position) {
    final Angle angle = position.getMeasure();

    switch (constants.SteerMotorClosedLoopOutput) {
      case TorqueCurrentFOC -> steerTalon.setControl(positionTorqueCurrent.withPosition(angle));
      case Voltage -> steerTalon.setControl(positionVoltage.withPosition(angle));
    }
  }

  @Override
  public void stopDriveMotor() {
    driveTalon.stopMotor();
  }

  @Override
  public void updateInputs(ModuleInputs inputs) {
    inputs.driveMotorConnected =
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveMotorVoltage,
                driveStatorCurrent,
                driveTorqueCurrent)
            .isOK();
    inputs.steerMotorConnected =
        BaseStatusSignal.refreshAll(
                steerPosition,
                steerVelocity,
                steerMotorVoltage,
                steerStatorCurrent,
                steerTorqueCurrent)
            .isOK();

    steerAbsolutePosition.refresh();
    inputs.steerEncoderConnected = steerAbsolutePosition.getStatus().isOK();

    inputs.drivePosition = drivePosition.getValue();
    inputs.driveVelocity = driveVelocity.getValue();
    inputs.driveMotorVoltage = driveMotorVoltage.getValue();
    inputs.driveStatorCurrent = driveStatorCurrent.getValue();
    inputs.driveTorqueCurrent = driveTorqueCurrent.getValue();

    inputs.facing = new Rotation2d(steerPosition.getValue());
    inputs.steerVelocity = steerVelocity.getValue();
    inputs.steerMotorVoltage = steerMotorVoltage.getValue();
    inputs.steerStatorCurrent = steerStatorCurrent.getValue();
    inputs.steerTorqueCurrent = steerTorqueCurrent.getValue();

    inputs.steerAbsolutePosition = steerAbsolutePosition.getValue();

    inputs.odometryAzimuthPositions = odometry.getAzimuthPositions();
    inputs.odometryWheelPositions =
        CommonUtils.mapToDouble(a -> a.in(Radians), odometry.getWheelPositions());
  }
}
