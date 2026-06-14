/*
 * Copyright (c) 2026 The Blue Devils.
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */
package com.frc6324.lib.util;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.Constants;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Collection;
import java.util.function.Supplier;

/** Utilities for working with the Phoenix 6 API. */
@UninstantiableClass
public final class PhoenixUtil {
  private static final Frequency SIGNAL_FREQUENCY = Hertz.of(100);
  private static final Lazy<SignalCache> signalCache = new Lazy<>(SignalCache::new);

  private PhoenixUtil() {
    throw new IllegalAccessError();
  }

  /**
   * Returns the average FPGA timestamp of the given signals.
   *
   * @param signals The signals whose timestamps will be included.
   * @return The average timestamp in the FPGA timebase.
   */
  public static double averageTimestamp(Collection<? extends BaseStatusSignal> signals) {
    int len = signals.size();
    if (len == 0) {
      return 0;
    }

    double totalTimestamp = 0;
    for (final BaseStatusSignal signal : signals) {
      totalTimestamp += signal.getTimestamp().getTime();
    }

    return Utils.currentTimeToFPGATime(totalTimestamp / len);
  }

  public static SignalCache getSignalCache() {
    return signalCache.get();
  }

  /**
   * Attempts to run a given command until it returns a code of OK.
   *
   * @param maxAttempts The maximum number of tries before this command gives up.
   * @param command The Phoenix 6 command to run.
   * @param message The message to print before the status code's description.
   * @return Whether the call succeeded
   */
  public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> command, String message) {
    StatusCode error = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < maxAttempts; i++) {
      error = command.get();

      if (error.isOK()) {
        return true;
      }
    }

    DriverStation.reportError(message + error.getDescription(), false);
    return false;
  }

  /**
   * Attempts to run a given command until it returns a code of OK.
   *
   * @param maxAttempts The maximum number of tries before this command gives up.
   * @param command The Phoenix 6 command to run.
   * @return Whether the call succeeded
   */
  public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    return tryUntilOk(maxAttempts, command, "Error trying to apply phoenix command: ");
  }

  public static class SignalCache {
    private static final StatusSignalCollection canivoreSignals = new StatusSignalCollection();

    private SignalCache() {}

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

    /**
     * Waits for every signal registered to the cache in a blocking call so that the CANivore can
     * synchronize them.
     *
     * @param timeout The maximum time to wait.
     */
    public static void synchronizeSignals(double timeout) {
      canivoreSignals.waitForAll(timeout);
    }

    /** Refreshes every signal in the CANivore status signal cache. */
    public static void refreshAllSignals() {
      canivoreSignals.refreshAll();
    }
  }
}
