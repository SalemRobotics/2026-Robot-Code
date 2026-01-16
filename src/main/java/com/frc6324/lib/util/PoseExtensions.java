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

import com.frc6324.lib.UninstantiableClass;
import edu.wpi.first.math.geometry.Pose2d;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

/**
 * Extensions for the {@link Pose2d} class for use with {@link lombok.experimental.ExtensionMethod
 * ExtensionMethod}.
 */
@UninstantiableClass
public final class PoseExtensions {
  @Contract(" -> fail")
  private PoseExtensions() {
    // Throw an Error since this means a reflection attack took place.
    throw new IllegalAccessError();
  }

  @Contract("_, _ -> new")
  public static @NotNull Pose2d plus(@NotNull Pose2d lhs, @NotNull Pose2d rhs) {
    return new Pose2d(
        lhs.getTranslation().plus(rhs.getTranslation()), lhs.getRotation().plus(rhs.getRotation()));
  }
}
