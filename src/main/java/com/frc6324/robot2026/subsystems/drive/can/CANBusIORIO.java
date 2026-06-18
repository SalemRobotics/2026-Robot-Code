package com.frc6324.robot2026.subsystems.drive.can;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;

public class CANBusIORIO implements CANBusIO {
  private final CANStatus status = new CANStatus();

  @Override
  public void updateInputs(CANBusInputs inputs) {
    CANJNI.getCANStatus(status);

    inputs.name = "rio";
    inputs.busUtilization = status.percentBusUtilization;
    inputs.busOffCount = status.busOffCount;
    inputs.txFullCount = status.txFullCount;
    inputs.receiveErrorCounter = status.receiveErrorCount;
    inputs.transmitErrorCounter = status.transmitErrorCount;
  }
}
