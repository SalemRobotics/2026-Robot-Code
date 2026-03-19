// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.frc6324.lib.util;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

@UninstantiableClass
public final class PhoenixUtil {
  private PhoenixUtil() {
    throw new IllegalAccessError();
  }

  private static final Frequency SIGNAL_FREQUENCY = Hertz.of(100);
  private static final StatusSignalCollection canivoreSignals = new StatusSignalCollection();

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command, String message) {
    StatusCode error = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < maxAttempts; i++) {
      error = command.get();

      if (error.isOK()) {
        return;
      }
    }

    DriverStation.reportError(message + error.getDescription(), false);
  }

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    tryUntilOk(maxAttempts, command, "Error trying to apply phoenix command: ");
  }

  /**
   * Adds a group of status signals to the robot's cache for more efficient refreshing.
   *
   * @param device The device the signals are on. This device must be on the robot's CANivore.
   * @param signals The signals to add to the cache.
   */
  public static void addSignals(ParentDevice device, BaseStatusSignal... signals) {
    if (device.getNetwork() != Constants.CANIVORE) {
      throw new IllegalArgumentException(
          "Signals added to PhoenixUtil must be on the CANivore bus.");
    }

    BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_FREQUENCY, signals);
    canivoreSignals.addSignals(signals);
  }

  public static void synchronizeSignals(double timeout) {
    canivoreSignals.waitForAll(timeout);
  }

  public static void refreshAllSignals() {
    canivoreSignals.refreshAll();
  }
}
