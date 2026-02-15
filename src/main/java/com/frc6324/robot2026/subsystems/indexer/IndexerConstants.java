package com.frc6324.robot2026.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;

@UninstantiableClass
public final class IndexerConstants {
  public static final CANBus INDEXER_CAN_BUS = Constants.CANIVORE;
  public static final int INDEXER_SPINNER_MOTOR_ID = 30;

  private IndexerConstants() {
    throw new IllegalAccessError();
  }
}
