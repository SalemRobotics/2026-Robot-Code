package com.frc6324.robot2026.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class CANBusIOCTRE implements CANBusIO {
  private final CANBus bus;

  @Override
  public void updateInputs(CANBusInputs inputs) {
    final CANBusStatus status = bus.getStatus();

    inputs.connected = status.Status.isOK();
    inputs.isCANFD = bus.isNetworkFD();

    inputs.utilization = status.BusUtilization;
    inputs.busOffCount = status.BusOffCount;
    inputs.txFullCount = status.TxFullCount;
    inputs.transmitErrorCount = status.TEC;
    inputs.receiveErrorCount = status.REC;
  }
}
