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
package com.frc6324.lib.odometry;

import com.frc6324.lib.odometry.struct.GyroReadingsStruct;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * A snapshot of IMU readings captured in a single robot loop.
 *
 * <p>Implements {@link StructSerializable} so that instances can be logged via Epilogue, published
 * over NetworkTables, or written to a WPILog without any additional boilerplate. The canonical
 * struct instance is available as {@link #struct}.
 */
public record GyroReadings(
    Rotation3d rotation, LinearAcceleration accelerationX, LinearAcceleration accelerationY)
    implements StructSerializable {

  /** Struct serializer for {@link GyroReadings}. Required by {@link StructSerializable}. */
  public static final GyroReadingsStruct struct = new GyroReadingsStruct();
}
