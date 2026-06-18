package com.frc6324.robot2026.subsystems.drive;

import static com.frc6324.robot2026.generated.TunerConstants.*;
import static edu.wpi.first.units.Units.*;

import com.frc6324.robot2026.generated.TunerConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import lombok.experimental.UtilityClass;

@UtilityClass
public final class DriveConstants {
  public static final Frequency ODOMETRY_FREQUENCY = Hertz.of(250);

  public static final double ODOMETRY_WAIT_PERIOD = ODOMETRY_FREQUENCY.asPeriod().in(Seconds) / 5;
  public static final double SYNCHRONIZATION_WAIT_PERIOD = 2;

  public static final int SIMULATION_TICKS_PER_LOOP = (int) (ODOMETRY_FREQUENCY.in(Hertz) / 50);

  public static final Vector<N3> ODOMETRY_STDDEVS =
      VecBuilder.fill(0.002, 0.002, Units.degreesToRadians(0.25));
  public static final Vector<N3> DEFAULT_VISION_STDDEVS =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  public static final String[] MODULE_NAMES = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

  public static final Translation2d[] MODULE_TRANSLATIONS = {
    new Translation2d(FrontLeft.LocationX, FrontLeft.LocationY),
    new Translation2d(FrontRight.LocationX, FrontRight.LocationY),
    new Translation2d(BackLeft.LocationX, BackLeft.LocationY),
    new Translation2d(BackRight.LocationX, BackRight.LocationY)
  };

  public static final Mass ROBOT_MASS = Pounds.of(140);
  public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6.83);
  public static final double WHEEL_COF = 2.1;
  public static final Distance WHEEL_RADIUS = Inches.of(2);

  public static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.25);
  public static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.35);

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(FrontLeft.LocationX, FrontLeft.LocationY),
              Math.hypot(FrontRight.LocationX, FrontRight.LocationY)),
          Math.max(
              Math.hypot(BackLeft.LocationX, BackLeft.LocationY),
              Math.hypot(BackRight.LocationX, BackRight.LocationY)));

  public static final LinearVelocity MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts;
  public static final double MAX_LINEAR_SPEED_METERS_PER_SEC = MAX_LINEAR_SPEED.in(MetersPerSecond);
  public static final AngularVelocity MAX_ANGULAR_SPEED =
      RadiansPerSecond.of(MAX_LINEAR_SPEED_METERS_PER_SEC / DRIVE_BASE_RADIUS);
  public static final double MAX_ANGULAR_SPEED_RADS_PER_SEC =
      MAX_ANGULAR_SPEED.in(RadiansPerSecond);
}
