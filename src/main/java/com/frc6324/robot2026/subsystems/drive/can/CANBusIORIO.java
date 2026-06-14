package com.frc6324.robot2026.subsystems.drive.can;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class CANBusIORIO implements CANBusIO {
  private final CANBus bus;

  @Override
  public void updateInputs(CANBusInputs inputs) {
    final CANBusStatus status = bus.getStatus();

    inputs.name = bus.getName();
    inputs.busUtilization = status.BusUtilization;
    inputs.busOffCount = status.BusOffCount;
    inputs.txFullCount = status.TxFullCount;
    inputs.receiveErrorCounter = status.REC;
    inputs.transmitErrorCounter = status.TEC;
  }
}
