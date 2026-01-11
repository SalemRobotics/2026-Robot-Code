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

import java.util.function.Supplier;
import lombok.Getter;
import org.jetbrains.annotations.Contract;

/**
 * Utility class that allows lazily loading a value with a provided initializer.
 *
 * @param <T> The type of the value being stored
 * @see Supplier
 */
public class Lazy<T> implements Supplier<T> {
  private T value = null;
  @Getter private final Supplier<T> supplier;

  /**
   * Creates a new lazily-initialized value.
   *
   * @param supplier The initializer for the value. This will stop being called by this class once it returns a nonnull value.
   */
  @Contract(pure = true)
  public Lazy(Supplier<T> supplier) {
    this.supplier = supplier;
  }

  /** Returns whether the value has been initialized yet. */
  public boolean isInitialized() {
    return value != null;
  }

  /** Forcibly initializes the value */
  public void initialize() {
    if (value == null) {
      value = supplier.get();
    }
  }

  /** Gets or initialize the lazily-initialized value. */
  @Override
  public T get() {
    initialize();

    return value;
  }
}
