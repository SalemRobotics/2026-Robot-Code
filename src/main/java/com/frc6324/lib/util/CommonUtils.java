package com.frc6324.lib.util;

import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.UninstantiableClass;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.jetbrains.annotations.Contract;

@UninstantiableClass
public final class CommonUtils {
  public static final LinearAccelerationUnit MetersPerSecSquared = MetersPerSecondPerSecond;

  public static final Angle NINETY_DEGREES = Degrees.of(90);

  /**
   * Gets the distance a wheel with the given radius has traveled, given this rotation is the amount
   * it has turned.
   *
   * @param angle The amount the wheel has turned. This must be a cumulative angle!
   * @param radius The radius of the wheel.
   * @return The distance the wheel has traveled.
   */
  public static Distance getDistanceTraveled(Angle angle, Distance radius) {
    final double dist = radius.in(Meters) * angle.in(Radians);
    return Meters.of(dist);
  }

  /**
   * Gets the linear velocity of a spinning mechanism given the size of the wheel.
   *
   * @param velocity The speed of the mechanism.
   * @param radius The radius of the wheel.
   * @return The current linear velocity of the mechanism.
   */
  public static LinearVelocity getVelocity(AngularVelocity velocity, Distance radius) {
    final double dist = radius.in(Meters) * velocity.in(RadiansPerSecond);
    return MetersPerSecond.of(dist);
  }

  @Contract("-> fail")
  private CommonUtils() {
    throw new IllegalAccessError();
  }
}
