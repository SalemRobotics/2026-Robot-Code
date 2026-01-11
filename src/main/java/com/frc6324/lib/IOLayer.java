/*
 * Copyright (c) 2025 The Blue Devils.
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

import org.jetbrains.annotations.Contract;

@UninstantiableClass
public final class IOLayer {
  @Contract(" -> fail")
  private IOLayer() {
    // Throw an Error since this means a reflection attack took place.
    throw new IllegalAccessError();
  }

  /**
   * A replay implementation for an I/O layer. This only works if the I/O interface is a {@link
   * java.lang.FunctionalInterface functional interface}.
   *
   * <p>an example of proper usage looks like:
   *
   * <pre>
   * {@code new ExampleSubsystem(IOLayer::replay)}
   * </pre>
   *
   * <p>Use of this function is preferable to something like:
   *
   * <pre>
   * {@code new ExampleSubsystem(new ExampleIOLayer() {}) }
   * </pre>
   *
   * @implNote This function does not actually do anything and is functionally equivalent to
   *     <pre>{@code (inputs) -> {}}</pre>
   *     However, this allows us to avoid allocating multiple lambda objects.
   * @param <T> The type of the loggable inputs
   * @param value The inputs a real or simulated I/O abstraction would modify; these are ignored in
   *     replay so they can accurately be restored from a log file.
   */
  @Contract(pure = true)
  public static <T> void replay(T value) {}
}
