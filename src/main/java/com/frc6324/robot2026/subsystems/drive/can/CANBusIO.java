package com.frc6324.robot2026.subsystems.drive.can;

import com.frc6324.lib.util.logging.IOLayer;
import org.littletonrobotics.junction.AutoLog;

public interface CANBusIO extends IOLayer<CANBusIO.CANBusInputs> {
  @AutoLog
  public class CANBusInputs {
    public String name = "";
    public double busUtilization = 0;
    public int busOffCount = 0;
    public int txFullCount = 0;
    public int receiveErrorCounter = 0;
    public int transmitErrorCounter = 0;
  }
}
