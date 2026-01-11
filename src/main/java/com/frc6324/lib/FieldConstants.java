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

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.jetbrains.annotations.Contract;

@UninstantiableClass
public class FieldConstants {
    @Contract(" -> fail")
    private FieldConstants() {
        // Throw an Error since this means a reflection attack took place.
        throw new IllegalAccessError();
    }

    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.2);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317.7);
    public static final Translation2d FIELD_CENTER = new Translation2d(FIELD_LENGTH_METERS / 2, FIELD_WIDTH_METERS / 2);
}
