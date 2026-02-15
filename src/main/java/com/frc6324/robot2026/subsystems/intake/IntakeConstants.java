package com.frc6324.robot2026.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

@UninstantiableClass
public final class IntakeConstants {
  public static final CANBus INTAKE_CAN_BUS = Constants.CANIVORE;
  public static final int INTAKE_MOTOR_ID = 20;

  public static final Distance INTAKE_WIDTH = Inches.of(27);
  public static final Distance INTAKE_EXTENSION = Inches.of(12);

  // Deployment setpoints
  public static final Angle INTAKE_DEPLOYED_POSITION = Rotations.of(1);
  public static final Angle INTAKE_DEPLOY_TOLERANCE = Degrees.of(2);
  public static final Angle INTAKE_STOWED_POSITION = Rotations.of(0);

  // TalonFX configurations
  public static final CurrentLimitsConfigs CURRENT_LIMITS =
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(70))
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimit(Amps.of(45))
          .withSupplyCurrentLimitEnable(true);

  public static final Slot0Configs DEPLOY_GAINS =
      new Slot0Configs().withKP(80).withKD(2).withKV(1).withKA(0.25).withKS(0.1);
  public static final Slot1Configs SPRING_GAINS =
      new Slot1Configs().withKP(5).withKD(0.001).withKS(0.01);
  public static final MotionMagicConfigs MOTION_MAGIC =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(25))
          .withMotionMagicJerk(0);

  public static final TalonFXConfiguration INTAKE_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(CURRENT_LIMITS)
          .withSlot0(DEPLOY_GAINS)
          .withSlot1(SPRING_GAINS)
          .withMotionMagic(MOTION_MAGIC);
  public static final int INTAKE_DEPLOY_SLOT = 0;
  public static final int INTAKE_SPRING_SLOT = 1;

  // Simulation constants for the extension/retraction motor
  public static final double INTAKE_MOI = 0.2;
  public static final double INTAKE_REDUCTION = 9;
  public static final DCMotor INTAKE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final MotorType INTAKE_MOTOR_TYPE = MotorType.KrakenX60;
  public static final Rotation3d INTAKE_MECHANISM_ROTATION =
      new Rotation3d(Degrees.zero(), Degrees.of(18.5), Degrees.zero());

  private IntakeConstants() {
    throw new IllegalAccessError();
  }
}
