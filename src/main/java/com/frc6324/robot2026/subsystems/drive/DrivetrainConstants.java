package com.frc6324.robot2026.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.UninstantiableClass;
import com.frc6324.lib.util.Statics;
import com.frc6324.robot2026.generated.TunerConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

@UninstantiableClass
public final class DrivetrainConstants {
  private DrivetrainConstants() {
    throw new IllegalAccessError();
  }

  public static final double ODOMETRY_UPDATE_FREQUENCY = 250;
  // Lower the odometry frequency for sim so that a laptop doesn't explode
  public static final int SIMULATION_TICKS_PER_LOOP = 5;

  public static final Vector<N3> ODOMETRY_STDDEVS =
      VecBuilder.fill(
          Units.inchesToMeters(0.07), Units.inchesToMeters(0.07), Units.degreesToRadians(0.25));
  public static final Vector<N3> DEFAULT_VISION_STDDEVS =
      VecBuilder.fill(
          Units.inchesToMeters(2), Units.inchesToMeters(2), Units.degreesToRadians(7.5));

  public static final String[] MODULE_NAMES = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
  public static final Translation2d[] MODULE_TRANSLATIONS = {
    new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
    new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
    new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
    new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
  };

  public static final Mass ROBOT_MASS = Pounds.of(140);
  public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6);
  public static final double WHEEL_COF = 2.2;

  public static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.25);
  public static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.35);

  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(ROBOT_MASS)
          .withBumperSize(Inches.of(34), Inches.of(34))
          .withCustomModuleTranslations(MODULE_TRANSLATIONS)
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60Foc(1),
                  DCMotor.getKrakenX44Foc(1),
                  TunerConstants.kDriveGearRatio,
                  TunerConstants.kSteerGearRatio,
                  DRIVE_FRICTION_VOLTAGE,
                  STEER_FRICTION_VOLTAGE,
                  Inches.of(2),
                  KilogramSquareMeters.of(0.1),
                  WHEEL_COF));

  public static final RobotConfig PATHPLANNER_CONFIG =
      Statics.initOrDefault(
          RobotConfig::fromGUISettings,
          () ->
              new RobotConfig(
                  ROBOT_MASS,
                  ROBOT_MOI,
                  new ModuleConfig(
                      Inches.of(2),
                      TunerConstants.kSpeedAt12Volts,
                      WHEEL_COF,
                      DCMotor.getKrakenX60Foc(1),
                      TunerConstants.FrontLeft.DriveMotorInitialConfigs.CurrentLimits
                          .getStatorCurrentLimitMeasure(),
                      1),
                  MODULE_TRANSLATIONS),
          (e) ->
              DriverStation.reportError(
                  "Failed to load settings from PathPlanner GUI: " + e.getMessage(),
                  e.getStackTrace()));

  // Set sim to start in the field to prevent wall collisions
  public static final Pose2d SIM_STARTING_POSE = new Pose2d(3, 3, Rotation2d.kZero);

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));
}
