package com.frc6324.robot2026.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.generated.TunerConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;

@UninstantiableClass
public final class DrivetrainConstants {
  private DrivetrainConstants() {
    throw new IllegalAccessError();
  }

  public static final double ODOMETRY_UPDATE_FREQUENCY = 250;
  public static final int SIMULATION_TICKS_PER_LOOP = (int) (ODOMETRY_UPDATE_FREQUENCY / 50);
  public static final Time ODOMETRY_PERIOD = Hertz.of(ODOMETRY_UPDATE_FREQUENCY).asPeriod();

  public static final Vector<N3> ODOMETRY_STDDEVS =
      VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(2.5));
  public static final Vector<N3> DEFAULT_VISION_STDDEVS =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  public static final String[] MODULE_NAMES = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

  public static final Translation2d[] MODULE_TRANSLATIONS = {
    new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
    new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
    new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
    new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
  };

  public static final Mass ROBOT_MASS = Pounds.of(140);
  public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6);
  public static final double WHEEL_COF = 1.6;

  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG = DriveTrainSimulationConfig.Default()
    .withRobotMass(ROBOT_MASS)
    .withBumperSize(Inches.of(27), Inches.of(27))
    .withCustomModuleTranslations(MODULE_TRANSLATIONS)
    .withGyro(COTS.ofPigeon2())
    .withSwerveModule(
      new SwerveModuleSimulationConfig(
        DCMotor.getKrakenX60Foc(1), 
        DCMotor.getKrakenX44Foc(1), 
        6.03, 
        287 / 11,
        Volts.of(0.1), 
        Volts.of(0.2), 
        Inches.of(2), 
        KilogramSquareMeters.of(0.03), 
        WHEEL_COF));

  public static final Pose2d STARTING_POSE = new Pose2d(3, 3, Rotation2d.kZero);

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  public static final PIDController BLINE_TRANSLATION_CONTROLLER = new PIDController(4.5, 0, 0);
  public static final PIDController BLINE_ROTATION_CONTROLLER = new PIDController(5, 0, 0);
  public static final PIDController BLINE_CTE_CONTROLLER = new PIDController(7, 0, 0);
}
