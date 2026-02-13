package com.frc6324.lib.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.frc6324.lib.UninstantiableClass;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

@UninstantiableClass
public final class UnitsUtils {
  public static final LinearAccelerationUnit MetersPerSecSquared = Units.MetersPerSecondPerSecond;

  /**
   * Gets the distance a wheel with the given diameter has traveled, given this rotation is the
   * amount it has turned.
   *
   * @param lhs The amount the wheel has turned. This must be a cumulative angle!
   * @param diameter The diameter of the wheel.
   * @return The distance the wheel has traveled.
   */
  public static Distance getDistanceTraveled(Angle lhs, Distance diameter) {
    final double dist = diameter.in(Meters) * lhs.in(Radians);
    return Meters.of(dist);
  }

  private UnitsUtils() {
    throw new IllegalAccessError();
  }
}
