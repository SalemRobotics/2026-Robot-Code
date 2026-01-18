/*
 * Copyright (c) 2025 The Blue Devils.
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
package com.frc6324.robot2026;

import com.ctre.phoenix6.CANBus;
import com.frc6324.lib.UninstantiableClass;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Constants for the whole robot.
 */
@UninstantiableClass
public final class Constants {
  public static final CANBus RIO_BUS = new CANBus("rio");
  public static final CANBus CANIVORE = new CANBus("canivore0");

  public static final int DRIVER_CONTROLLER_PORT = 0;

  /** The mode that should be run when the robot is being simulated. */
  public static final Mode SIM_MODE = Mode.SIM;

  /** The current mode being run. */
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

  private Constants() {
    throw new IllegalAccessError();
  }

  /** Represents a type of robot being run. */
  public static enum Mode {
    /** Running on the real robot. */
    REAL,
    /** Running a robot simulation. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }
}
