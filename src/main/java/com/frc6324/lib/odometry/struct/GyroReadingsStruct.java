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
package com.frc6324.lib.odometry.struct;

import com.frc6324.lib.odometry.GyroReadings;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

/**
 * Raw struct serialization for {@link GyroReadings}.
 *
 * <h2>Wire layout</h2>
 *
 * <pre>
 * ┌─────────────────────────────────────────────────────────────┐
 * │  rotation  (Rotation3d → Quaternion: w, x, y, z)  32 bytes  │
 * │  accelerationX  (float64, m/s²)                    8 bytes  │
 * │  accelerationY  (float64, m/s²)                    8 bytes  │
 * └─────────────────────────────────────────────────────────────┘
 *                                               Total: 48 bytes
 * </pre>
 *
 * <p>{@link Rotation3d} delegates to {@link edu.wpi.first.math.geometry.Quaternion#struct}, which
 * packs four {@code float64} values (w, x, y, z) in that order. The two {@link
 * edu.wpi.first.units.measure.LinearAcceleration} fields carry no struct of their own; they are
 * stored as raw {@code float64} values in SI base units (meters per second squared), matching the
 * convention used by WPILib for all other unit-typed fields (e.g. {@code Rotation2dStruct} stores
 * its angle in radians).
 */
public class GyroReadingsStruct implements Struct<GyroReadings> {
  /** Size in bytes: Rotation3d (via Quaternion) + two float64 acceleration values. */
  public static final int SIZE = Rotation3d.struct.getSize() + 2 * Struct.kSizeDouble;

  @Override
  public Class<GyroReadings> getTypeClass() {
    return GyroReadings.class;
  }

  @Override
  public String getTypeName() {
    return "GyroReadings";
  }

  @Override
  public int getSize() {
    return SIZE;
  }

  /**
   * Schema string consumed by WPILib's struct descriptor system.
   *
   * <p>{@code Rotation3d rotation} is a nested struct, so its type name appears verbatim. The two
   * acceleration fields are primitive {@code float64} values tagged with the SI unit so that tools
   * like Glass and Epilogue can display correct units.
   */
  @Override
  public String getSchema() {
    return "Rotation3d rotation;" + "float64 accelerationX_mps2;" + "float64 accelerationY_mps2";
  }

  /**
   * Declares the nested struct dependency so that WPILib's schema registry can resolve the {@code
   * Rotation3d} type referenced in {@link #getSchema()}.
   */
  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Rotation3d.struct};
  }

  /**
   * {@link GyroReadings} is a Java {@code record}: all its components are final and it has
   * value-based equality. Declaring it immutable lets WPILib skip defensive copies when the struct
   * is used in log buffers and NetworkTables publishers.
   */
  @Override
  public boolean isImmutable() {
    return true;
  }

  /**
   * Deserializes a {@link GyroReadings} from the current position of {@code bb}.
   *
   * <p>Field order must match {@link #pack}: rotation, accelerationX, accelerationY.
   */
  @Override
  public GyroReadings unpack(ByteBuffer bb) {
    Rotation3d rotation = Rotation3d.struct.unpack(bb);
    double accelXMps2 = bb.getDouble();
    double accelYMps2 = bb.getDouble();
    return new GyroReadings(
        rotation,
        Units.MetersPerSecondPerSecond.of(accelXMps2),
        Units.MetersPerSecondPerSecond.of(accelYMps2));
  }

  /**
   * Serializes {@code value} into the current position of {@code bb}.
   *
   * <p>Field order must match {@link #unpack}: rotation, accelerationX, accelerationY.
   */
  @Override
  public void pack(ByteBuffer bb, GyroReadings value) {
    Rotation3d.struct.pack(bb, value.rotation());
    bb.putDouble(value.accelerationX().in(Units.MetersPerSecondPerSecond));
    bb.putDouble(value.accelerationY().in(Units.MetersPerSecondPerSecond));
  }
}
