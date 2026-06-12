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

import java.util.ArrayList;
import java.util.List;

/**
 * A base class for subsystems that do not utilize command-based utilities.
 *
 * <p>This should only be used for subsystems like vision or LEDs, since they only need to run code
 * periodically rather than in commands.
 */
public abstract class VirtualSubsystem {
  private static final List<VirtualSubsystem> SUBSYSTEMS = new ArrayList<>();

  /** Creates and registers a new virtual subsystem. */
  protected VirtualSubsystem() {
    SUBSYSTEMS.add(this);
  }

  /** Runs the periodic function for each virtual subsystem on the robot. */
  public static void allPeriodics() {
    SUBSYSTEMS.forEach(VirtualSubsystem::periodic);
  }

  /** Code that will run every time {@link VirtualSubsystem#allPeriodics()} is called. */
  public abstract void periodic();
}
