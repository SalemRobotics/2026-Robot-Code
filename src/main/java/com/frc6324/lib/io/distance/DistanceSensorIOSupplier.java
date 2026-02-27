package com.frc6324.lib.io.distance;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DistanceSensorIOSupplier implements DistanceSensorIO {
  private final Supplier<Distance> distanceSupplier;

  public DistanceSensorIOSupplier(DoubleSupplier supplier) {
    this(() -> Meters.of(supplier.getAsDouble()));
  }

  public DistanceSensorIOSupplier(Supplier<Distance> distanceSupplier) {
    this.distanceSupplier = distanceSupplier;
  }

  @Override
  public void updateInputs(DistanceSensorInputs inputs) {
    inputs.connected = true;
    inputs.distanceStddev = Meters.zero();
    inputs.detectedDistance = distanceSupplier.get();
  }
}
