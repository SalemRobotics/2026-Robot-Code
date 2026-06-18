package com.frc6324.robot2026.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import lombok.experimental.UtilityClass;

@UtilityClass
public final class IntakeConstants {
  public static final CANBus INTAKE_CAN_BUS = Constants.CANIVORE;
  public static final int INTAKE_MOTOR_ID = 20;
  public static final double INTAKE_REDUCTION = 9;

  public static final int INTAKE_FAST_SLOT = 0;
  public static final int INTAKE_RETRACT_SLOT = 1;
  public static final int INTAKE_EFFICIENCY_SLOT = 2;

  public static final Distance INTAKE_WIDTH = Inches.of(27);
  public static final Distance INTAKE_EXTENSION = Inches.of(12);

  // Deployment setpoints
  public static final Angle INTAKE_MAX_POSITION = Rotations.of(4.1);
  public static final Angle INTAKE_DEPLOYED_POSITION = Rotations.of(4.1);
  public static final Angle INTAKE_VISION_THRESHOLD = Rotations.of(2);
  public static final Angle INTAKE_DEPLOY_TOLERANCE = Degrees.of(60);
  public static final Angle INTAKE_RETRACTED_POSITION = Rotations.of(1.3);
  public static final Angle INTAKE_TRENCH_SAFE_POSITION = Rotations.of(2.8);
  public static final Angle INTAKE_STOWED_POSITION = Rotations.of(0);

  public static final TalonFXConfiguration INTAKE_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(100))
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(INTAKE_REDUCTION))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(RotationsPerSecond.of(2.4))
                  .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(50))
                  .withMotionMagicJerk(0))
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
          .withSlot0(new Slot0Configs().withKP(200).withKI(10).withKD(6.7).withKV(1).withKS(0.1))
          .withSlot1(new Slot1Configs().withKP(60).withKD(6.7).withKV(1).withKA(0.25).withKS(0.1))
          .withSlot2(new Slot2Configs().withKP(80).withKD(6.7).withKV(1).withKA(0.25).withKS(0.1))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitThreshold(INTAKE_MAX_POSITION)
                  .withForwardSoftLimitEnable(true)
                  .withReverseSoftLimitThreshold(INTAKE_STOWED_POSITION)
                  .withReverseSoftLimitEnable(true));

  // Simulation constants for the extension/retraction motor
  public static final double INTAKE_MOI = 0.2;
  public static final DCMotor INTAKE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final MotorType INTAKE_MOTOR_TYPE = MotorType.KrakenX60;
  public static final Rotation3d INTAKE_MECHANISM_ROTATION =
      new Rotation3d(Degrees.zero(), Degrees.of(18.5), Degrees.zero());
}
