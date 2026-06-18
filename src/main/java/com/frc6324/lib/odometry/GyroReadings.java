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
