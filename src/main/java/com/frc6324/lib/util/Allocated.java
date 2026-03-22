package com.frc6324.lib.util;

import java.util.function.Supplier;

public final class Allocated<T> implements Supplier<T> {
  private T value;

  public Allocated(T value) {
    set(value);
  }

  @Override
  public T get() {
    return value;
  }

  public void set(T newValue) {
    value = newValue;
  }
}
