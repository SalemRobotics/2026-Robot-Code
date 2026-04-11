package com.frc6324.robot2026.subsystems.drive;

import com.frc6324.lib.util.IOLayer;
import org.littletonrobotics.junction.AutoLog;

public interface CANBusIO extends IOLayer<CANBusIO.CANBusInputs> {
  @AutoLog
  public class CANBusInputs {
    public boolean connected;
    public boolean isCANFD;
    public double utilization;

    public int busOffCount;
    public int txFullCount;
    public int transmitErrorCount;
    public int receiveErrorCount;
  }
}
