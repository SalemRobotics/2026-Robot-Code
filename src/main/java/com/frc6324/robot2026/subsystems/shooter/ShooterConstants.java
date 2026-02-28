package com.frc6324.robot2026.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

@UninstantiableClass
public final class ShooterConstants {
  public static final CANBus SHOOTER_CAN_BUS = Constants.CANIVORE;
  public static final Transform2d SHOOTER_POSITION =
      new Transform2d(Units.inchesToMeters(-6), Units.inchesToMeters(6), Rotation2d.kZero);

  public static final double APPROX_FUEL_PER_SECOND = 1;
  public static final double TIME_TO_LAUNCH_FUEL = (1 / APPROX_FUEL_PER_SECOND);

  public static final InterpolatingTreeMap<Double, Angle> HOOD_ANGLE_MAP =
      new InterpolatingTreeMap<>(
          InverseInterpolator.forDouble(), (a1, a2, t) -> a1.plus(a2.minus(a1).times(t)));

  public static final InterpolatingTreeMap<Double, AngularVelocity> HUB_FLYWHEEL_VELOCITY_MAP =
      new InterpolatingTreeMap<>(
          InverseInterpolator.forDouble(), (v1, v2, t) -> v1.plus(v2.minus(v1).times(t)));

  public static final InterpolatingTreeMap<Double, AngularVelocity> PASSING_FLYWHEEL_VELOCITY_MAP =
      new InterpolatingTreeMap<>(
          InverseInterpolator.forDouble(), (v1, v2, t) -> v1.plus(v2.minus(v1).times(t)));

  static {
    HUB_FLYWHEEL_VELOCITY_MAP.put(0.0, RotationsPerSecond.of(34));
    HUB_FLYWHEEL_VELOCITY_MAP.put(1.2, RotationsPerSecond.of(34));
    HUB_FLYWHEEL_VELOCITY_MAP.put(1.5, RotationsPerSecond.of(35.25));
    HUB_FLYWHEEL_VELOCITY_MAP.put(1.75, RotationsPerSecond.of(36.75));
    HUB_FLYWHEEL_VELOCITY_MAP.put(2.0, RotationsPerSecond.of(37.75));
    HUB_FLYWHEEL_VELOCITY_MAP.put(2.25, RotationsPerSecond.of(39));
    HUB_FLYWHEEL_VELOCITY_MAP.put(2.5, RotationsPerSecond.of(40.25));
    HUB_FLYWHEEL_VELOCITY_MAP.put(2.75, RotationsPerSecond.of(41.5));
    HUB_FLYWHEEL_VELOCITY_MAP.put(3.0, RotationsPerSecond.of(42.75));
    HUB_FLYWHEEL_VELOCITY_MAP.put(3.25, RotationsPerSecond.of(44));
    HUB_FLYWHEEL_VELOCITY_MAP.put(3.5, RotationsPerSecond.of(45.25));
    HUB_FLYWHEEL_VELOCITY_MAP.put(3.75, RotationsPerSecond.of(45.25));
    HUB_FLYWHEEL_VELOCITY_MAP.put(4.0, RotationsPerSecond.of(46.25));
    HUB_FLYWHEEL_VELOCITY_MAP.put(4.25, RotationsPerSecond.of(47));
    HUB_FLYWHEEL_VELOCITY_MAP.put(4.5, RotationsPerSecond.of(47.75));
    HUB_FLYWHEEL_VELOCITY_MAP.put(5.0, RotationsPerSecond.of(48.75));
    HUB_FLYWHEEL_VELOCITY_MAP.put(5.3, RotationsPerSecond.of(50.5));

    // TODO: velocities from 2 -> 6 (steps of 0.25)

    // TODO: find correspondence between hood angles & velocities at distance
    // (map should have a point for every velocity map entry)
    HOOD_ANGLE_MAP.put(0.0, Rotations.of(0.15));
    HOOD_ANGLE_MAP.put(3.5, Rotations.of(0.15));
    HOOD_ANGLE_MAP.put(3.75, Rotations.of(0.225));
    HOOD_ANGLE_MAP.put(4.0, Rotations.of(0.3));
    HOOD_ANGLE_MAP.put(4.25, Rotations.of(0.3));
    HOOD_ANGLE_MAP.put(4.5, Rotations.of(0.35));
    HOOD_ANGLE_MAP.put(5.0, Rotations.of(0.3875));
    HOOD_ANGLE_MAP.put(5.3, Rotations.of(0.4));
  }

  /** Constants for the shooter's hood. */
  @UninstantiableClass
  public static final class HoodConstants {
    public static final int HOOD_MOTOR_ID = 40;
    public static final double HOOD_REDUCTION = 5;

    /** The translation from the robot's center to the axle the hood is mounted on. */
    public static final Translation3d ROBOT_TO_HOOD_AXLE =
        new Translation3d(Inches.of(-1.75), Inches.of(0.75), Inches.of(17.5));

    // CAD centroid: -110.334 mm, -22.4069 mm, 60.9053 mm

    /** The translation from the axle the hood is mounted on to the hood itself. */
    public static final Translation3d HOOD_AXLE_TO_HOOD =
        new Translation3d(-0.110334, 0, -0.0609053);

    /**
     * The offset from the {@link #HOOD_AXLE_TO_HOOD} translation that gets the center point of the
     * hood.
     */
    public static final Translation3d HOOD_SHOOTING_OFFSET = new Translation3d(0, -0.0224069, 0);

    // Setpoint values & tolerance
    public static final Angle HOOD_STOW_ANGLE = Degrees.zero();
    public static final Angle HOOD_MAX_ANGLE = Degrees.of(25);
    public static final Angle HOOD_TOLERANCE = Degrees.of(2.5);

    /** The current limits for the hood's motor. */
    public static final CurrentLimitsConfigs HOOD_CURRENT_LIMITS =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(70))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(45))
            .withSupplyCurrentLimitEnable(true);

    /** Configuration values for the hood motor's Motion Magic&reg; controls. */
    public static final MotionMagicConfigs HOOD_MOTION_MAGIC =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5))
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    /** The PID gains for the hood motor to reach target positions. */
    public static final Slot0Configs HOOD_GAINS =
        new Slot0Configs().withKP(50).withKI(5).withKD(1).withKG(2.6);

    /** The feedback information (gear ratios) for the hood's motor. */
    public static final FeedbackConfigs HOOD_FEEDBACK =
        new FeedbackConfigs().withSensorToMechanismRatio(HOOD_REDUCTION);

    /** The full configuration for the hood's {@link com.ctre.phoenix6.hardware.TalonFX TalonFX}. */
    public static final TalonFXConfiguration HOOD_MOTOR_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(HOOD_CURRENT_LIMITS)
            .withSlot0(new Slot0Configs().withKP(175).withKI(10).withKD(6).withKG(2.6))
            .withMotionMagic(HOOD_MOTION_MAGIC)
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(HOOD_FEEDBACK);

    /** The gearbox for the hood's motor. */
    public static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX44Foc(1);

    /** The type of motor the hood uses (a Kraken X44). */
    public static final MotorType HOOD_MOTOR_TYPE = MotorType.KrakenX44;

    /** The inertia of the hood. */
    public static final MomentOfInertia HOOD_MOI = KilogramSquareMeters.of(0.02);

    private HoodConstants() {
      throw new IllegalAccessError();
    }
  }

  @UninstantiableClass
  public static final class FlywheelConstants {
    public static final int FLYWHEEL_LEADER_ID = 41;
    public static final int FLYWHEEL_FOLLOWER_ID = 42;
    // Accounts for loss of speed due to compression, slip and others that we can't explicitly model
    // in sim
    public static final double FLYWHEEL_EFFICIENCY = 0.7;
    public static final double FLYWHEEL_BACKSPIN_EFFICIENCY = 0.4;
    public static final Distance FLYWHEEL_RADIUS = Inches.of(1.5);

    public static final AngularVelocity FLYWHEEL_VELOCITY_TOLERANCE = RadiansPerSecond.one();

    public static final MotorAlignmentValue FLYWHEEL_MOTOR_ALIGNMENT = MotorAlignmentValue.Opposed;

    public static final CurrentLimitsConfigs FLYWHEEL_CURRENT_LIMITS =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(120))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(100))
            .withSupplyCurrentLimitEnable(true);

    public static final Slot0Configs FLYWHEEL_GAINS =
        new Slot0Configs().withKP(10).withKI(5).withKD(0.1).withKS(10).withKV(0.175);

    public static final TalonFXConfiguration FLYWHEEL_MOTOR_CONFIG =
        new TalonFXConfiguration()
            .withSlot0(FLYWHEEL_GAINS)
            .withCurrentLimits(FLYWHEEL_CURRENT_LIMITS);

    public static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getKrakenX60Foc(2);
    public static final MotorType FLYWHEEL_MOTOR_TYPE = MotorType.KrakenX60;
    public static final MomentOfInertia FLYWHEEL_MOI = KilogramSquareMeters.of(0.0011705586);

    private FlywheelConstants() {
      throw new IllegalAccessError();
    }
  }

  private ShooterConstants() {
    throw new IllegalAccessError();
  }
}
