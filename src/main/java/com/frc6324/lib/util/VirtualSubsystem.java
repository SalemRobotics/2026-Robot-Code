package com.frc6324.lib.util;

import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem {
  private static final List<VirtualSubsystem> SUBSYSTEMS = new ArrayList<>();

  protected VirtualSubsystem() {
    SUBSYSTEMS.add(this);
  }

  public static void allPeriodics() {
    SUBSYSTEMS.forEach(VirtualSubsystem::periodic);
  }

  public abstract void periodic();
}
