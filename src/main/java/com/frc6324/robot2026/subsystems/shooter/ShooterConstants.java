package com.frc6324.robot2026.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

@UninstantiableClass
public final class ShooterConstants {
  public static final CANBus SHOOTER_CAN_BUS = Constants.CANIVORE;

  public static final InterpolatingTreeMap<Double, HubShotParams> HUB_SHOT_MAP =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), HubShotParams::interpolate);
  public static final InterpolatingTreeMap<Double, AngularVelocity> PASSING_VELOCITY_MAP =
      new InterpolatingTreeMap<>(
          InverseInterpolator.forDouble(), (v1, v2, t) -> v1.plus(v2.minus(v1).times(t)));

  public static final Transform2d SHOOTER_POSITION =
      new Transform2d(Units.inchesToMeters(-7), 0, Rotation2d.k180deg);

  public static final double TIME_TO_LAUNCH_FUEL = 0.04;

  static {
    // TODO: tune tree maps
  }

  private ShooterConstants() {
    throw new IllegalAccessError();
  }

  @UninstantiableClass
  public final class AcceleratorConstants {
    public static final int ACCELERATOR_MOTOR_ID = 40;
    public static final double ACCELERATOR_REDUCTION = 2.43;

    public static final TalonFXConfiguration ACCELERATOR_MOTOR_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(ACCELERATOR_REDUCTION))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withControlTimesyncFreqHz(Hertz.of(100))
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(
                new Slot0Configs()
                    .withKP(10)
                    .withKS(2)
                    .withKV(1)
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign));

    public static final DCMotor ACCELERATOR_GEARBOX = DCMotor.getKrakenX44Foc(1);
    public static final MotorType ACCELERATOR_MOTOR_TYPE = MotorType.KrakenX44;
    public static final double ACCELERATOR_MOI = 0.05;

    private AcceleratorConstants() {
      throw new IllegalAccessError();
    }
  }

  @UninstantiableClass
  public final class DrumConstants {
    public static final int DRUM_LEADER_ID = 41;
    public static final int[] DRUM_MOTOR_IDS = new int[] {42, 43, 44};
    public static final InvertedValue[] DRUM_FOLLOWER_DIRECTIONS =
        new InvertedValue[] {
          InvertedValue.CounterClockwise_Positive,
          InvertedValue.Clockwise_Positive,
          InvertedValue.Clockwise_Positive
        };
    // Accounts for loss of speed due to compression, slip and others that we can't explicitly model
    // in sim
    public static final double DRUM_EFFICIENCY = 0.7;
    public static final double DRUM_BACKSPIN_EFFICIENCY = 0.4;
    public static final Distance DRUM_RADIUS = Inches.of(2);

    public static final AngularVelocity DRUM_IDLE_SPEED = RPM.of(2000);
    public static final AngularVelocity DRUM_CLOSE_HUB_SHOT_SPEED = RPM.of(2100);
    public static final MotorAlignmentValue DRUM_MOTOR_ALIGNMENT = MotorAlignmentValue.Opposed;

    public static final TalonFXConfiguration DRUM_MOTOR_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(150))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(60))
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withControlTimesyncFreqHz(Hertz.of(500))
                    .withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs().withKP(9).withKI(5).withKD(0.15).withKS(10).withKV(0.2))
            .withSlot1(new Slot1Configs().withKP(5).withKI(5).withKS(0.15).withKS(10).withKV(0.2));

    public static final int DRUM_SHOOTING_SLOT = 0;
    public static final int DRUM_SPINUP_SLOT = 1;

    public static final DCMotor DRUM_GEARBOX = DCMotor.getKrakenX60Foc(4);
    public static final MotorType DRUM_MOTOR_TYPE = MotorType.KrakenX60;
    public static final double DRUM_MOI = 0.0011705586;

    private DrumConstants() {
      throw new IllegalAccessError();
    }
  }

  @UninstantiableClass
  public final class HoodConstants {
    public static final int HOOD_MOTOR_ID = 45;
    public static final double HOOD_REDUCTION = 5 * 10.4;

    public static final Angle HOOD_MAX_ANGLE = Rotations.one();
    public static final Angle HOOD_STOW_ANGLE = Rotations.of(0.03);
    public static final Angle HOOD_POSITION_TOLERANCE = Rotations.of(0.005);

    public static final TalonFXConfiguration HOOD_MOTOR_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(100))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(30))
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(HOOD_REDUCTION))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withControlTimesyncFreqHz(Hertz.of(100))
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(
                new Slot0Configs()
                    .withKP(500)
                    .withKI(0)
                    .withKD(6)
                    .withKS(0.02)
                    .withKV(0)
                    .withKG(0.45)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));

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

    public static final Angle HOOD_SIM_MAX_ANGLE = Degrees.of(45);

    public static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX44Foc(1);
    public static final MotorType HOOD_MOTOR_TYPE = MotorType.KrakenX44;
    public static final double HOOD_MOI = 0.05;

    private HoodConstants() {
      throw new IllegalAccessError();
    }
  }

  /**
   * Parameters for a shot into the hub.
   *
   * @param hoodAngle The target angle of the shooter's hood.
   * @param drumVelocity The target velocity of the drum.
   */
  public record HubShotParams(Angle hoodAngle, AngularVelocity drumVelocity)
      implements Interpolatable<HubShotParams> {
    @Override
    public HubShotParams interpolate(HubShotParams endValue, double t) {
      if (t == 0) {
        return this;
      } else if (t == 1) {
        return endValue;
      } else {
        final Angle newHoodAngle = hoodAngle.plus(endValue.hoodAngle.minus(hoodAngle).times(t));
        final AngularVelocity newDrumVelocity =
            drumVelocity.plus(endValue.drumVelocity.minus(drumVelocity).times(t));

        return new HubShotParams(newHoodAngle, newDrumVelocity);
      }
    }
  }
}
