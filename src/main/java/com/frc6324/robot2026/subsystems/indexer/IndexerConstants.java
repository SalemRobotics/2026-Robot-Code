package com.frc6324.robot2026.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import lombok.experimental.UtilityClass;

@UtilityClass
public final class IndexerConstants {
  public static final CANBus INDEXER_CAN_BUS = Constants.CANIVORE;
  public static final int INDEXER_BELT_MOTOR_ID = 30;
  public static final int INDEXER_FEEDER_MOTOR_ID = 31;

  public static final AngularVelocity INDEXER_BELT_VELOCITY = RPM.of(600);
  public static final AngularVelocity INDEXER_BELT_SHAKE_ADDITION = RPM.of(5);
  public static final AngularVelocity INDEXER_KICKER_VELOCITY = RotationsPerSecond.of(38);

  public static final TalonFXConfiguration INDEXER_BELT_CONFIG =
      new TalonFXConfiguration()
          // CURRENT LIMITS
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLimitEnable(true))
          // FEEDBACK
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(3))
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
          // GAINS
          .withSlot0(new Slot0Configs().withKP(25).withKD(0.5).withKV(0.05).withKS(2.6));

  public static final TalonFXConfiguration INDEXER_KICKER_CONFIG =
      new TalonFXConfiguration()
          // CURRENT LIMITS
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(40)))
          // FEEDBACK
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(3))
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          // GAINS
          .withSlot0(new Slot0Configs().withKP(10).withKV(0.175).withKS(5));

  public static final DCMotor INDEXER_KICKER_GEARBOX = DCMotor.getKrakenX44Foc(1);
  public static final MotorType INDEXER_KICKER_MOTOR_TYPE = MotorType.KrakenX44;
  public static final double INDEXER_KICKER_REDUCTION = 3;
  public static final double INDEXER_KICKER_MOI = 0.01;

  public static final DCMotor INDEXER_BELT_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final MotorType INDEXER_BELT_MOTOR_TYPE = MotorType.KrakenX60;
  public static final double INDEXER_BELT_REDUCTION = 9;
  public static final double INDEXER_BELT_MOI = 0.1;
}
