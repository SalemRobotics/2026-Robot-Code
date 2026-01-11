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

package com.frc6324.lib;


import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import org.jetbrains.annotations.NotNull;

import static edu.wpi.first.units.Units.Seconds;

public final class DeltaTimeCalculator {
    private double lastTimestamp = RobotController.getFPGATime() / 1e6;

    public double get() {
        double newTimestamp = RobotController.getFPGATime() / 1e6;
        double delta = newTimestamp - lastTimestamp;
        lastTimestamp = newTimestamp;
        return delta;
    }

    public @NotNull Time getMeasure() {
        return Seconds.of(get());
    }
}
