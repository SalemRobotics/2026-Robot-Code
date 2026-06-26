package com.frc6324.robot2026.sim;

import com.frc6324.lib.util.Lazy;

public class FuelSimulator {
  private static final Lazy<FuelSimulator> INSTANCE = new Lazy<>(FuelSimulator::new);

  private FuelSimulator() {}

  public static FuelSimulator getInstance() {
    return INSTANCE.get();
  }

  public int heldFuelCount() {
    return 1;
  }
}
