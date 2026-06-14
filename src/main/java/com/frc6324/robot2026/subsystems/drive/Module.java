package com.frc6324.robot2026.subsystems.drive;

import static com.frc6324.robot2026.subsystems.drive.DriveConstants.*;
import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.util.CommonUtils;
import com.frc6324.robot2026.subsystems.drive.module.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod(CommonUtils.class)
public class Module {
  private final String name;
  private final ModuleIO io;
  private final ModuleInputsAutoLogged inputs = new ModuleInputsAutoLogged();

  private final Alert driveDisconnectedAlert;
  private final Alert steerDisconnectedAlert;
  private final Alert encoderDisconnectedAlert;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[0];

  public Module(int index, ModuleIO io) {
    this.name = MODULE_NAMES[index];
    this.io = io;

    driveDisconnectedAlert =
        new Alert("Drivetrain", name + " drive motor is disconnected.", AlertType.kError);
    steerDisconnectedAlert =
        new Alert("Drivetrain", name + " steer motor is disconnected.", AlertType.kError);
    encoderDisconnectedAlert =
        new Alert("Drivetrain", name + " encoder is disconnected.", AlertType.kError);
  }

  private static double convertDrivePositionToMeters(double position) {
    return position * WHEEL_RADIUS.in(Meters);
  }

  public Angle getDrivePosition() {
    return inputs.drivePosition;
  }

  public AngularVelocity getDriveVelocity() {
    return inputs.driveVelocity;
  }

  public SwerveModulePosition getMeasuredPosition() {
    return new SwerveModulePosition(getWheelPosition(), getModuleFacing());
  }

  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(getWheelVelocity(), getModuleFacing());
  }

  public Rotation2d getModuleFacing() {
    return inputs.facing;
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public Distance getWheelPosition() {
    return getDrivePosition().getDistanceTraveled(WHEEL_RADIUS);
  }

  public LinearVelocity getWheelVelocity() {
    return getDriveVelocity().getVelocity(WHEEL_RADIUS);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/" + name, inputs);

    int sampleCount = inputs.odometryWheelPositions.length;
    odometryPositions = new SwerveModulePosition[sampleCount];

    for (int i = 0; i < sampleCount; i++) {
      final double position = convertDrivePositionToMeters(inputs.odometryWheelPositions[i]);
      final Rotation2d angle = inputs.odometryAzimuthPositions[i];

      odometryPositions[i] = new SwerveModulePosition(position, angle);
    }

    if (DriverStation.isDisabled()) {
      stop();
    }

    driveDisconnectedAlert.set(!inputs.driveMotorConnected);
    steerDisconnectedAlert.set(!inputs.steerMotorConnected);
    encoderDisconnectedAlert.set(!inputs.steerEncoderConnected);
  }

  public void runDriveCharacterization(Current current) {
    io.setDriveCurrent(current);
    io.setSteerCurrent(Amps.zero());
  }

  public void runDriveCharacterization(Voltage voltage) {
    io.setDriveVoltage(voltage);
    io.setSteerVoltage(Volts.zero());
  }

  public void runSetpoint(SwerveModuleState setpoint) {
    setpoint.optimize(getModuleFacing());
    setpoint.cosineScale(getModuleFacing());

    final double desiredSpeed = setpoint.speedMetersPerSecond / WHEEL_RADIUS.in(Meters);

    io.setDriveVelocity(RadiansPerSecond.of(desiredSpeed));
    io.setSteerPosition(setpoint.angle);
  }

  public void runSetpoint(
      SwerveModuleState setpoint,
      LinearAcceleration acceleration,
      Force feedforwardX,
      Force feedforwardY) {
    setpoint.optimize(getModuleFacing());
    setpoint.cosineScale(getModuleFacing());

    final double desiredMotorVelocity = setpoint.speedMetersPerSecond / WHEEL_RADIUS.in(Meters);
    final double desiredAcceleration =
        acceleration.in(MetersPerSecondPerSecond) / WHEEL_RADIUS.in(Meters);

    final Translation2d force2d =
        new Translation2d(feedforwardX.in(Newtons), feedforwardY.in(Newtons));
    // Cosine scale the force vector
    final double moduleFeedforwardForce =
        force2d.getNorm() * force2d.getAngle().minus(getModuleFacing()).getCos();
    final double moduleFeedforwardTorque = moduleFeedforwardForce * WHEEL_RADIUS.in(Meters);

    io.setDriveVelocity(
        RadiansPerSecond.of(desiredMotorVelocity),
        RadiansPerSecondPerSecond.of(desiredAcceleration),
        NewtonMeters.of(moduleFeedforwardTorque));
    io.setSteerPosition(setpoint.angle);
  }

  public void stop() {
    io.stopDriveMotor();
    io.setSteerVoltage(Volts.zero());
  }
}
