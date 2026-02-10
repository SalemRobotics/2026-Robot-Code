package com.frc6324.robot2026.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

@UninstantiableClass
public final class ShooterConstants {
  public static final CANBus SHOOTER_CAN_BUS = Constants.CANIVORE;

  public static final int SHOOTER_HOOD_MOTOR_ID = 30;
  public static final int SHOOTER_FLYWHEEL_MOTOR_ID = 31;

  public static final Translation3d ROBOT_TO_HOOD_AXLE =
      new Translation3d(
          Units.inchesToMeters(-1.75), Units.inchesToMeters(0.75), Units.inchesToMeters(15));
  // CAD centroid: -110.334 mm, -22.4069 mm, 60.9053 mm
  public static final Translation3d HOOD_AXLE_TO_HOOD = new Translation3d(-0.110334, 0, -0.0609053);

  public static final Angle SHOOTER_HOOD_STOW_ANGLE = Degrees.zero();
  public static final Angle SHOOTER_HOOD_MAX_ANGLE = Degrees.of(25);

  public static final DCMotor SHOOTER_HOOD_GEARBOX = DCMotor.getKrakenX44Foc(1);
  public static final MotorType SHOOTER_HOOD_MOTOR_TYPE = MotorType.KrakenX44;
  public static final double SHOOTER_HOOD_MOI = 0.02;
  public static final double SHOOTER_HOOD_REDUCTION = 9;

  public static final CurrentLimitsConfigs SHOOTER_HOOD_CURRENT_LIMITS =
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(70))
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimit(Amps.of(45))
          .withSupplyCurrentLimitEnable(true);
  public static final MotionMagicConfigs HOOD_MOTION_MAGIC =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1))
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5))
          .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
  public static final Slot0Configs SHOOTER_HOOD_GAINS =
      new Slot0Configs().withKP(225).withKI(10).withKD(15).withKS(2.5).withKV(4).withKA(2);
  public static final FeedbackConfigs SHOOTER_HOOD_FEEDBACK =
      new FeedbackConfigs().withSensorToMechanismRatio(SHOOTER_HOOD_REDUCTION);

  public static final TalonFXConfiguration SHOOTER_HOOD_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(SHOOTER_HOOD_CURRENT_LIMITS)
          .withSlot0(SHOOTER_HOOD_GAINS)
          .withMotionMagic(HOOD_MOTION_MAGIC)
          .withFeedback(SHOOTER_HOOD_FEEDBACK);

  public static final Slot0Configs SHOOTER_FLYWHEEL_GAINS = new Slot0Configs().withKP(60).withKD(6);
  public static final TalonFXConfiguration SHOOTER_FLYWHEEL_MOTOR_CONFIG =
      new TalonFXConfiguration().withSlot0(SHOOTER_FLYWHEEL_GAINS);

  public static final DCMotor SHOOTER_FLYWHEEL_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final MotorType SHOOTER_FLYWHEEL_MOTOR_TYPE = MotorType.KrakenX60;
  public static final MomentOfInertia SHOOTER_FLYWHEEL_MOI =
      Pounds.mult(InchesPerSecond).mult(Inches).per(RadiansPerSecond).of(4);
  public static final double SHOOTER_FLYWHEEL_REDUCTION = 1;

  private ShooterConstants() {
    throw new IllegalAccessError();
  }
}
