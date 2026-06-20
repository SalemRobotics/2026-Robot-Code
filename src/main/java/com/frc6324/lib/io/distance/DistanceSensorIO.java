package com.frc6324.lib.io.distance;

import static edu.wpi.first.units.Units.Meters;

import com.frc6324.lib.io.IOLayer;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface DistanceSensorIO extends IOLayer<DistanceSensorIO.DistanceSensorInputs> {
  default boolean isActive() {
    return false;
  }

  @AutoLog
  public class DistanceSensorInputs {
    public boolean connected = false;

    public Distance detectedDistance = Meters.zero();
    public Distance distanceStddev = Meters.zero();
  }
}
