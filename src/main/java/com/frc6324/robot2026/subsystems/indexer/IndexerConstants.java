package com.frc6324.robot2026.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;

@UninstantiableClass
public final class IndexerConstants {
  public static final CANBus INDEXER_CAN_BUS = Constants.CANIVORE;
  public static final int INDEXER_SPINNER_MOTOR_ID = 30;
  public static final int INDEXER_FEEDER_MOTOR_ID = 31;

  public static final AngularVelocity INDEXER_SPINNER_VELOCITY = RPM.of(450);
  public static final AngularVelocity INDEXER_SPINNER_SHAKE_ADDITION = RPM.of(5);
  public static final AngularVelocity INDEXER_KICKER_VELOCITY = RotationsPerSecond.of(38);

  public static final TalonFXConfiguration INDEXER_SPINNER_CONFIG =
      new TalonFXConfiguration()
          // CURRENT LIMITS
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(80))
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLimitEnable(true))
          // FEEDBACK
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(9))
          // GAINS
          .withSlot0(new Slot0Configs().withKP(25).withKD(0.5).withKV(0.05).withKS(2.6));

  public static final TalonFXConfiguration INDEXER_KICKER_CONFIG =
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
          // GAINS
          .withSlot0(new Slot0Configs().withKP(6).withKV(0.15).withKS(5));

  public static final DCMotor INDEXER_KICKER_GEARBOX = DCMotor.getKrakenX44Foc(1);
  public static final MotorType INDEXER_KICKER_MOTOR_TYPE = MotorType.KrakenX44;
  public static final double INDEXER_KICKER_REDUCTION = 3;
  public static final double INDEXER_KICKER_MOI = 0.01;

  public static final DCMotor INDEXER_SPINNER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final MotorType INDEXER_SPINNER_MOTOR_TYPE = MotorType.KrakenX60;
  public static final double INDEXER_SPINNER_REDUCTION = 9;
  public static final double INDEXER_SPINNER_MOI = 0.1;

  private IndexerConstants() {
    throw new IllegalAccessError();
  }
}
