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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import org.jetbrains.annotations.NotNull;

/** A utility class for calculating accurate time deltas. */
public final class DeltaTimeCalculator {
  private double lastTimestamp = RobotController.getFPGATime() / 1e6;

  /**
   * Gets the delta from now to the last time this method was called.
   *
   * @return The amount of time (in seconds) since the last time this method was called.
   */
  public double get() {
    double newTimestamp = RobotController.getFPGATime() / 1e6;
    double delta = newTimestamp - lastTimestamp;
    lastTimestamp = newTimestamp;
    return delta;
  }

  /**
   * Equivalent to {@link #get}, but returns a measure in the Units API.
   *
   * @return A measure of time since the last time this method or {@link #get} was called.
   * @see #get()
   * @see Time
   */
  public @NotNull Time getMeasure() {
    return Seconds.of(get());
  }
}
