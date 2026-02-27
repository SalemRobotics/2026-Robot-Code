package com.frc6324.robot2026.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;

@UninstantiableClass
public final class IndexerConstants {
  public static final CANBus INDEXER_CAN_BUS = Constants.CANIVORE;
  public static final int INDEXER_SPINNER_MOTOR_ID = 30;
  public static final int INDEXER_FEEDER_MOTOR_ID = 31;

  public static final CurrentLimitsConfigs SPINNER_CURRENT_LIMITS =
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(60))
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimit(Amps.of(40))
          .withSupplyCurrentLimitEnable(true);

  public static final Slot0Configs SPINNER_GAINS =
      new Slot0Configs().withKP(5).withKD(1).withKV(1).withKS(1);

  public static final TalonFXConfiguration INDEXER_SPINNER_CONFIG = new TalonFXConfiguration();

  public static final CurrentLimitsConfigs KICKER_CURRENT_LIMITS =
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(60))
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimit(Amps.of(40))
          .withSupplyCurrentLimitEnable(true);

  public static final Slot0Configs KICKER_GAINS =
      new Slot0Configs().withKP(5).withKD(1).withKV(1).withKS(1);

  public static final TalonFXConfiguration INDEXER_KICKER_CONFIG =
      new TalonFXConfiguration().withCurrentLimits(KICKER_CURRENT_LIMITS).withSlot0(KICKER_GAINS);

  private IndexerConstants() {
    throw new IllegalAccessError();
  }
}
