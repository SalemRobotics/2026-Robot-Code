package com.frc6324.robot2026.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.math.geometry.*;
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

  public static final double APPROX_FUEL_PER_SECOND = 5;
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

    PASSING_FLYWHEEL_VELOCITY_MAP.put(0.0, RotationsPerSecond.of(20));
    PASSING_FLYWHEEL_VELOCITY_MAP.put(3.4, RotationsPerSecond.of(42));
    PASSING_FLYWHEEL_VELOCITY_MAP.put(4.1, RotationsPerSecond.of(45));
    PASSING_FLYWHEEL_VELOCITY_MAP.put(4.75, RotationsPerSecond.of(47));
    PASSING_FLYWHEEL_VELOCITY_MAP.put(5.5, RotationsPerSecond.of(50));
    PASSING_FLYWHEEL_VELOCITY_MAP.put(6.0, RotationsPerSecond.of(55));
    PASSING_FLYWHEEL_VELOCITY_MAP.put(7.0, RotationsPerSecond.of(59));
    PASSING_FLYWHEEL_VELOCITY_MAP.put(8.8, RotationsPerSecond.of(61.5));
    PASSING_FLYWHEEL_VELOCITY_MAP.put(16.0, RotationsPerSecond.of(61.5));

    HOOD_ANGLE_MAP.put(0.0, Rotations.of(0.15));
    HOOD_ANGLE_MAP.put(3.5, Rotations.of(0.15));
    HOOD_ANGLE_MAP.put(3.75, Rotations.of(0.225));
    HOOD_ANGLE_MAP.put(4.0, Rotations.of(0.3));
    HOOD_ANGLE_MAP.put(4.25, Rotations.of(0.3));
    HOOD_ANGLE_MAP.put(4.5, Rotations.of(0.35));
    HOOD_ANGLE_MAP.put(5.0, Rotations.of(0.3875));
    HOOD_ANGLE_MAP.put(7.0, Rotations.of(0.4));
  }

  /** Constants for the shooter's hood. */
  @UninstantiableClass
  public static final class HoodConstants {
    public static final int HOOD_MOTOR_ID = 40;
    public static final double HOOD_REDUCTION = 5;

    /** The translation from the robot's center to the axle the hood is mounted on. */
    public static final Translation3d ROBOT_TO_HOOD_AXLE =
        new Translation3d(Inches.of(-1.75), Inches.of(0.75), Inches.of(17.5));

    /** The translation from the axle the hood is mounted on to the hood itself. */
    public static final Translation3d HOOD_AXLE_TO_HOOD =
        new Translation3d(-0.110334, 0, -0.0609053);

    /**
     * The offset from the {@link #HOOD_AXLE_TO_HOOD} translation that gets the center point of the
     * hood.
     */
    public static final Translation3d HOOD_SIM_SHOOTING_OFFSET =
        new Translation3d(0, -0.0224069, 0);

    // Setpoint values & tolerance
    public static final Angle HOOD_STOW_ANGLE = Rotations.of(0.05);
    public static final Angle HOOD_MAX_ANGLE = Rotations.of(0.6);
    public static final Angle HOOD_SIM_MAX_ANGLE = Degrees.of(25);
    public static final Angle HOOD_TOLERANCE = Rotations.of(0.05);

    /** The full configuration for the hood's {@link com.ctre.phoenix6.hardware.TalonFX TalonFX}. */
    public static final TalonFXConfiguration HOOD_MOTOR_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(70))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(HOOD_REDUCTION))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withControlTimesyncFreqHz(Hertz.of(100))
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs().withKP(200).withKI(15).withKD(3.5).withKG(2.6))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitThreshold(HOOD_MAX_ANGLE)
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(Rotations.zero())
                    .withReverseSoftLimitEnable(true));

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

    public static final AngularVelocity FLYWHEEL_IDLE_SPEED = RPM.of(1500);
    public static final MotorAlignmentValue FLYWHEEL_MOTOR_ALIGNMENT = MotorAlignmentValue.Opposed;

    public static final TalonFXConfiguration FLYWHEEL_MOTOR_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(100))
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withControlTimesyncFreqHz(Hertz.of(100))
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs().withKP(10).withKI(5).withKD(0.1).withKS(10).withKV(0.175))
            .withSlot1(new Slot1Configs().withKP(5).withKI(5).withKS(0.05).withKS(10).withKV(0.75));

    public static final int FLYWHEEL_SHOOTING_SLOT = 0;
    public static final int FLYWHEEL_SPINUP_SLOT = 1;

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
