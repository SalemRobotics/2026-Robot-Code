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

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

/** */
@UninstantiableClass
public final class AllianceFlipUtil {
  @Contract(" -> fail")
  private AllianceFlipUtil() {
    // Throw an Error since this means a reflection attack took place.
    throw new IllegalAccessError();
  }

  /**
   * @param x The X coordinate to flip.
   * @return The corresponding X coordinate on the other side of the field.
   */
  @Contract(pure = true)
  public static double applyXUnchecked(double x) {
    return FieldConstants.FIELD_LENGTH_METERS - x;
  }

  /**
   * Conditionally flips the provided X coordinate based on the current alliance.
   *
   * @param x The X coordinate to flip.
   * @return The corresponding X coordinate on the other alliance, unless the current alliance is
   *     blue.
   */
  public static double applyX(double x) {
    return shouldFlip() ? applyXUnchecked(x) : x;
  }

  /**
   * @param y The Y coordinate to flip.
   * @return The corresponding Y coordinate on the other side of the field.
   */
  @Contract(pure = true)
  public static double applyYUnchecked(double y) {
    return FieldConstants.FIELD_WIDTH_METERS - y;
  }

  /**
   * Conditionally flips the provided Y coordinate based on the current alliance.
   *
   * @param y The Y coordinate to flip.
   * @return The corresponding Y coordinate on the other alliance, unless the current alliance is
   *     blue.
   */
  public static double applyY(double y) {
    return shouldFlip() ? applyYUnchecked(y) : y;
  }

  @Contract("_ -> new")
  public static @NotNull Translation2d applyUnchecked(@NotNull Translation2d translation) {
    return new Translation2d(
        applyXUnchecked(translation.getX()), applyYUnchecked(translation.getX()));
  }

  public static Translation2d apply(Translation2d translation) {
    return shouldFlip() ? applyUnchecked(translation) : translation;
  }

  @Contract("_ -> new")
  public static @NotNull Rotation2d applyUnchecked(@NotNull Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.k180deg);
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? applyUnchecked(rotation) : rotation;
  }

  @Contract("_ -> new")
  public static @NotNull Pose2d applyUnchecked(@NotNull Pose2d pose) {
    return new Pose2d(applyUnchecked(pose.getTranslation()), applyUnchecked(pose.getRotation()));
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip() ? applyUnchecked(pose) : pose;
  }

  @Contract("_ -> new")
  public static @NotNull Transform2d applyUnchecked(@NotNull Transform2d transform) {
    return new Transform2d(-transform.getX(), -transform.getY(), transform.getRotation().times(-1));
  }

  public static Transform2d apply(Transform2d transform) {
    return shouldFlip() ? applyUnchecked(transform) : transform;
  }

  @Contract("_ -> new")
  public static @NotNull Translation3d apply(@NotNull Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  public static Rotation3d apply(Rotation3d rotation) {
    return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  @Contract("_ -> new")
  public static @NotNull Pose3d apply(@NotNull Pose3d pose) {
    return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  /**
   * @return Whether poses should be flipped to the red alliance.
   */
  public static boolean shouldFlip() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }
}
