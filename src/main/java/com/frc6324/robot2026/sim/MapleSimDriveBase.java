package com.frc6324.robot2026.sim;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.sim.*;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public final class MapleSimDriveBase {
  private final Pigeon2SimState pigeonSim;
  private final SimSwerveModule[] simModules;
  private final SwerveDriveSimulation simulation;

  @SuppressWarnings("unchecked")
  public MapleSimDriveBase(
      Pigeon2 pigeon,
      SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
          moduleConstants) {
    this.pigeonSim = pigeon.getSimState();
    this.simulation = MapleSimManager.getInstance().getMainRobotDriveSimulation();
    simModules = new SimSwerveModule[moduleConstants.length];

    SwerveModuleSimulation[] moduleSimulations = simulation.getModules();
    for (int i = 0; i < this.simModules.length; i++) {
      simModules[i] = new SimSwerveModule(moduleConstants[i], moduleSimulations[i], modules[i]);
    }
  }

  public ChassisSpeeds getChassisSpeeds() {
    return simulation.getDriveTrainSimulatedChassisSpeedsRobotRelative();
  }

  public Pose2d getPose() {
    return simulation.getSimulatedDriveTrainPose();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    simulation.setSimulationWorldPose(pose);
  }

  public void update() {
    pigeonSim.setRawYaw(getRotation().getMeasure());
    pigeonSim.setAngularVelocityZ(RadiansPerSecond.of(getChassisSpeeds().omegaRadiansPerSecond));
  }

  /** Represents a simulation of a single {@link SwerveModule}. */
  protected static class SimSwerveModule {
    public final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        moduleConstants;
    public final SwerveModuleSimulation moduleSimulation;

    /**
     * Creates a new {@link SwerveModule} simulation.
     *
     * @param moduleConstants
     * @param moduleSimulation
     * @param module
     */
    public SimSwerveModule(
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            moduleConstants,
        SwerveModuleSimulation moduleSimulation,
        SwerveModule<TalonFX, TalonFX, CANcoder> module) {
      this.moduleConstants = moduleConstants;
      this.moduleSimulation = moduleSimulation;
      moduleSimulation.useDriveMotorController(
          new TalonFXMotorControllerSim(module.getDriveMotor()));
      moduleSimulation.useSteerMotorController(
          new TalonFXMotorControllerWithRemoteCanCoderSim(
              module.getSteerMotor(), module.getEncoder()));
    }
  }

  // Static utils classes
  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    public final int id;

    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.id = talonFX.getDeviceID();
      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  public static class TalonFXMotorControllerWithRemoteCanCoderSim
      extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCancoderSimState;

    public TalonFXMotorControllerWithRemoteCanCoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.remoteCancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      remoteCancoderSimState.setRawPosition(mechanismAngle);
      remoteCancoderSimState.setVelocity(mechanismVelocity);

      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }
}
