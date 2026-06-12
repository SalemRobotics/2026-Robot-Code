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

import com.frc6324.lib.UninstantiableClass;
import edu.wpi.first.math.geometry.Translation2d;
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

  /**
   * Checks if two polygons intersect or otherwise overlap each other.
   *
   * @param shape1 The first shape's corners.
   * @param shape2 The second shape's corners.
   * @return Whether an intersection is present.
   */
  public static boolean polygonsIntersect(Translation2d[] shape1, Translation2d[] shape2) {
    return !(hasSeparatingAxis(shape1, shape2) || hasSeparatingAxis(shape2, shape1));
  }

  private static boolean hasSeparatingAxis(Translation2d[] shape1, Translation2d[] shape2) {
    for (int i = 0; i < shape1.length; i++) {
      final Translation2d p1 = shape1[i];
      final Translation2d p2 = shape1[(i + 1) % shape1.length];

      final double edgeX = p2.getX() - p1.getX();
      final double edgeY = p2.getY() - p1.getY();

      final double axisX = -edgeY;
      final double axisY = edgeX;

      final double[] proj1 = projectPolygon(shape1, axisX, axisY);
      final double[] proj2 = projectPolygon(shape2, axisX, axisY);

      if (proj1[1] < proj2[0] || proj2[1] < proj1[0]) {
        return true;
      }
    }
    return false;
  }

  private static double[] projectPolygon(Translation2d[] shape, double axisX, double axisY) {
    double min = Double.POSITIVE_INFINITY;
    double max = Double.NEGATIVE_INFINITY;

    for (final Translation2d p : shape) {
      final double projection = p.getX() * axisX + p.getY() * axisY;
      min = Math.min(min, projection);
      max = Math.max(max, projection);
    }

    return new double[] {min, max};
  }

  @Contract("-> fail")
  private CommonUtils() {
    throw new IllegalAccessError();
  }
}
