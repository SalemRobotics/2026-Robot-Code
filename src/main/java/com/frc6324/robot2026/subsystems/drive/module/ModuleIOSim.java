package com.frc6324.robot2026.subsystems.drive.module;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.frc6324.lib.util.CommonUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ModuleIOSim implements ModuleIO {
  private static final double DRIVE_KP = 0.12;
  private static final double STEER_KP = 8.0;
  private static final double STEER_KD = 0.0;

  private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, ?> constants;

  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor, steerMotor;
  private final DCMotor driveMechanismGearbox;

  private boolean runDriveClosedLoop = false;
  private boolean runSteerClosedLoop = false;

  private final PIDController driveController = new PIDController(DRIVE_KP, 0, 0);
  private final PIDController steerController = new PIDController(STEER_KP, 0, STEER_KD);

  private double driveAppliedVoltage = 0.0;
  private double steerAppliedVoltage = 0.0;

  private double desiredWheelVelocityRadPerSec = 0.0;
  private double torqueFeedForwardVoltage = 0.0;
  private Rotation2d desiredSteerFacing = Rotation2d.kZero;

  public ModuleIOSim(
      SwerveModuleSimulation moduleSimulation,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, ?> moduleConstants) {
    this.moduleSimulation = moduleSimulation;
    this.driveMotor =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(
                Amps.of(moduleConstants.DriveMotorInitialConfigs.CurrentLimits.StatorCurrentLimit));
    this.steerMotor =
        moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(
                Amps.of(moduleConstants.SteerMotorInitialConfigs.CurrentLimits.StatorCurrentLimit));

    this.constants = moduleConstants;
    this.driveMechanismGearbox =
        DCMotor.getKrakenX60(1).withReduction(moduleConstants.DriveMotorGearRatio);

    // Enable wrapping for turn PID
    steerController.enableContinuousInput(-Math.PI, Math.PI);

    SimulatedArena.getInstance().addCustomSimulation((subTickNum) -> runControlLoops());
  }

  public void runControlLoops() {
    // Run control loops if activated
    if (runDriveClosedLoop) calculateDriveControlLoops();
    else driveController.reset();
    if (runSteerClosedLoop) calculateSteerControlLoops();
    else steerController.reset();

    // Feed voltage to motor simulation
    driveMotor.requestVoltage(Volts.of(driveAppliedVoltage));
    steerMotor.requestVoltage(Volts.of(steerAppliedVoltage));
  }

  private void calculateDriveControlLoops() {
    final double frictionTorque =
        driveMechanismGearbox.getTorque(
                driveMechanismGearbox.getCurrent(0, constants.DriveFrictionVoltage))
            * Math.signum(desiredWheelVelocityRadPerSec);
    final double velocityFeedforwardVolts =
        driveMechanismGearbox.getVoltage(frictionTorque, desiredWheelVelocityRadPerSec);
    final double feedforwardVolts = velocityFeedforwardVolts + torqueFeedForwardVoltage;
    final double feedBackVolts =
        driveController.calculate(
            moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond),
            desiredWheelVelocityRadPerSec);
    driveAppliedVoltage = feedforwardVolts + feedBackVolts;
  }

  private Voltage calculateFeedForwardVoltage(Torque ff) {
    final double velocity = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    final double voltage = driveMechanismGearbox.getVoltage(ff.in(NewtonMeters), velocity);
    return Volts.of(voltage);
  }

  private void calculateSteerControlLoops() {
    steerAppliedVoltage =
        steerController.calculate(
            moduleSimulation.getSteerAbsoluteFacing().getRadians(),
            desiredSteerFacing.getRadians());
  }

  @Override
  public void setDriveCurrent(Current current) {
    final DCMotor driveMotorModel = moduleSimulation.config.driveMotorConfigs.motor;
    this.driveAppliedVoltage =
        driveMotorModel.getVoltage(
            driveMotorModel.getCurrent(current.in(Amps)),
            moduleSimulation.getDriveEncoderUnGearedSpeed().in(RadiansPerSecond));
    this.runDriveClosedLoop = false;
    this.torqueFeedForwardVoltage = 0;
  }

  @Override
  public void setDriveVoltage(Voltage volts) {
    this.driveAppliedVoltage = volts.in(Volts);
    this.runDriveClosedLoop = false;
    this.torqueFeedForwardVoltage = 0;
  }

  @Override
  public void setSteerCurrent(Current current) {
    final DCMotor steerMotorModel = moduleSimulation.getSteerMotorConfigs().motor;
    this.steerAppliedVoltage =
        steerMotorModel.getVoltage(
            steerMotorModel.getCurrent(current.in(Amps)),
            moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond));
    this.runSteerClosedLoop = false;
  }

  @Override
  public void setSteerVoltage(Voltage volts) {
    this.steerAppliedVoltage = volts.in(Volts);
    this.runSteerClosedLoop = false;
  }

  @Override
  public void setDriveVelocity(AngularVelocity velocity) {
    this.desiredWheelVelocityRadPerSec = velocity.in(RadiansPerSecond);
    this.runDriveClosedLoop = true;
    this.torqueFeedForwardVoltage = 0;
  }

  @Override
  public void setDriveVelocity(
      AngularVelocity velocity, AngularAcceleration acceleration, Torque feedforward) {
    this.desiredWheelVelocityRadPerSec = velocity.in(RadiansPerSecond);
    this.runDriveClosedLoop = true;
    this.torqueFeedForwardVoltage = calculateFeedForwardVoltage(feedforward).in(Volts);
  }

  @Override
  public void setSteerPosition(Rotation2d facing) {
    this.desiredSteerFacing = facing;
    this.runSteerClosedLoop = true;
  }

  @Override
  public void updateInputs(ModuleInputs inputs) {
    inputs.driveMotorConnected = true;
    inputs.steerMotorConnected = true;
    inputs.steerEncoderConnected = true;

    inputs.drivePosition = moduleSimulation.getDriveWheelFinalPosition();
    inputs.driveVelocity = moduleSimulation.getDriveWheelFinalSpeed();
    inputs.driveMotorVoltage = moduleSimulation.getDriveMotorAppliedVoltage();
    inputs.driveStatorCurrent = moduleSimulation.getDriveMotorStatorCurrent();
    // Set torque current to stator current because maple-sim doesn't simulate torque current
    inputs.driveTorqueCurrent = inputs.driveStatorCurrent;

    inputs.facing = moduleSimulation.getSteerAbsoluteFacing();
    inputs.steerVelocity = moduleSimulation.getSteerAbsoluteEncoderSpeed();
    inputs.steerMotorVoltage = moduleSimulation.getSteerMotorAppliedVoltage();
    inputs.steerStatorCurrent = moduleSimulation.getSteerMotorStatorCurrent();
    // Same deal as drive torque current
    inputs.steerTorqueCurrent = inputs.steerStatorCurrent;

    inputs.odometryAzimuthPositions = moduleSimulation.getCachedSteerAbsolutePositions();
    inputs.odometryWheelPositions =
        CommonUtils.mapToDouble(
            a -> a.in(Radians), moduleSimulation.getCachedDriveWheelFinalPositions());
  }
}
