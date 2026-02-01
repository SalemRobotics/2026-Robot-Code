package com.frc6324.robot2026.subsystems.drive;

import static com.frc6324.robot2026.subsystems.drive.DrivetrainConstants.*;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public final class MapleSimDriveBase extends SwerveDriveSimulation {
  private final Pigeon2SimState pigeonSim;
  private final SimSwerveModule[] simModules;

  @SuppressWarnings("unchecked")
  public MapleSimDriveBase(
      Pigeon2 pigeon,
      SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
          moduleConstants) {
    super(MAPLE_SIM_CONFIG, STARTING_POSE);
    this.pigeonSim = pigeon.getSimState();
    simModules = new SimSwerveModule[moduleConstants.length];

    SwerveModuleSimulation[] moduleSimulations = getModules();
    for (int i = 0; i < this.simModules.length; i++) {
      simModules[i] = new SimSwerveModule(moduleConstants[0], moduleSimulations[i], modules[i]);
    }

    SimulatedArena.overrideSimulationTimings(ODOMETRY_PERIOD, 1);
    SimulatedArena.getInstance().addDriveTrainSimulation(this);
    if (SimulatedArena.getInstance() instanceof Arena2026Rebuilt rebuilt) {
      rebuilt.setEfficiencyMode(false);
    }
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void update() {
    SimulatedArena.getInstance().simulationPeriodic();
    pigeonSim.setRawYaw(getSimulatedDriveTrainPose().getRotation().getMeasure());

    final ChassisSpeeds robotSpeeds = getDriveTrainSimulatedChassisSpeedsRobotRelative();
    pigeonSim.setAngularVelocityZ(RadiansPerSecond.of(robotSpeeds.omegaRadiansPerSecond));
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
