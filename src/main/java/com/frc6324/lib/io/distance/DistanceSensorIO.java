package com.frc6324.lib.io.distance;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface DistanceSensorIO {
  default boolean isActive() {
    return false;
  }

  void updateInputs(DistanceSensorInputs inputs);

  @AutoLog
  public class DistanceSensorInputs {
    public boolean connected = false;

    public Distance detectedDistance = Meters.zero();
    public Distance distanceStddev = Meters.zero();
  }
}
