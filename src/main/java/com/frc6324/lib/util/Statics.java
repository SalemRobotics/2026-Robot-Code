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
package com.frc6324.lib.util;

import com.frc6324.lib.UninstantiableClass;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

/** Helper functions for initializing static variables in a type-safe exception-safe way. */
@UninstantiableClass
public final class Statics {
  @Contract(" -> fail")
  private Statics() {
    // Throw an Error since this means a reflection attack took place.
    throw new IllegalAccessError();
  }

  /**
   * The default exception handler used by {@link Statics#initOrDefault(ThrowableSupplier,
   * Supplier)}. This reports the error to DriverStation with the error's filled-in stack trace.
   *
   * @param <E> The type of exception being handled (this does not affect how the exception is
   *     handled).
   * @param error The error to handle.
   */
  public static <E extends Exception> void defaultExceptionHandler(@NotNull E error) {
    // Fill in the stack trace of the exception
    Throwable throwable = error.fillInStackTrace();

    // Print the error to the driver station.
    DriverStation.reportError("Error ocurred during static initialization: ", false);
    DriverStation.reportError(throwable.toString(), throwable.getStackTrace());
  }

  /**
   * Attempts to initialize a value with one constructor, using a fallback if it throws an error.
   *
   * @param <T> The type of the value being initialized.
   * @param init The constructor that may throw an exception.
   * @param defaultSupplier The default value to use if {@code init} throws something. This will run
   *     unchecked, so throwing an exception in this function will cause your robot program to crash
   *     if it is not properly handled.
   * @return The initialized value.
   */
  public static <T> T initOrDefault(
      ThrowableSupplier<T, Exception> init, Supplier<T> defaultSupplier) {
    // Use the default exception handler
    return initOrDefault(init, defaultSupplier, Statics::defaultExceptionHandler);
  }

  /**
   * Attempts to initialize a value with one constructor, using a given exception handler and
   * default supplier if it throws an exception.
   *
   * @param <T> The type of the value being initialized.
   * @param <E> The type of exception that may be thrown.
   * @param init The constructor that may throw an exception.
   * @param defaultSupplier The default value to use if {@code init} throws something. This will run
   *     unchecked, so throwing an exception in this function will cause your robot program to crash
   *     if it is not properly handled.
   * @param exceptionHandler The exception handler to use if an exception is encountered.
   * @return The initialized value.
   */
  @SuppressWarnings("unchecked")
  public static <T, E extends Exception> T initOrDefault(
      ThrowableSupplier<T, E> init, Supplier<T> defaultSupplier, Consumer<E> exceptionHandler) {
    try {
      // Try to get
      return init.get();
    } catch (Exception e) {
      // Handle the exception
      exceptionHandler.accept((E) e);

      // Return the value from the default supplier
      return defaultSupplier.get();
    }
  }

  /**
   * Equivalent to {@link Supplier}, however it may throw a checked exception.
   *
   * @param <T> The type of the value being returned.
   * @param <E> The type of exception that can be thrown. Setting this to {@link Exception} will
   *     allow any type of exception to be thrown.
   */
  @FunctionalInterface
  public interface ThrowableSupplier<T, E extends Exception> {
    /**
     * Gets the value, or throws an exception.
     *
     * @return The value returned by this function-like object
     * @throws E The type of exception that can be thrown.
     */
    T get() throws E;
  }
}
