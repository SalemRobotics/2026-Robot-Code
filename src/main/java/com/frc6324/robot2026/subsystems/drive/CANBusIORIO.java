package com.frc6324.robot2026.subsystems.drive;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;

public class CANBusIORIO implements CANBusIO {
  private final CANStatus status = new CANStatus();

  @Override
  public void updateInputs(CANBusInputs inputs) {
    CANJNI.getCANStatus(status);

    inputs.connected = true;
    inputs.isCANFD = false;

    inputs.utilization = status.percentBusUtilization;
    inputs.busOffCount = status.busOffCount;
    inputs.txFullCount = status.txFullCount;
    inputs.transmitErrorCount = status.transmitErrorCount;
    inputs.receiveErrorCount = status.receiveErrorCount;
  }
}
