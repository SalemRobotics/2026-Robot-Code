// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.frc6324.lib.util;

import com.ctre.phoenix6.StatusCode;
import com.frc6324.lib.UninstantiableClass;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

@UninstantiableClass
public final class PhoenixUtil {
  private PhoenixUtil() {
    throw new IllegalAccessError();
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command, String message) {
    StatusCode error = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < maxAttempts; i++) {
      error = command.get();

      if (error.isOK()) {
        return;
      }
    }

    DriverStation.reportError(message + error.getDescription(), false);
  }

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      final StatusCode status = command.get();
      if (status.isOK()) {
        break;
      }
    }
  }
}
