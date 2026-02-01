package com.frc6324.robot2026.subsystems.leds;

import com.frc6324.lib.UninstantiableClass;
import edu.wpi.first.wpilibj.util.Color;

@UninstantiableClass
public final class LEDsConstants {
  public static final int LED_RIO_PWM_PORT = 0;
  public static final int LED_BUFFER_LENGTH = 8;

  public static final Color LED_DEFAULT_COLOR = new Color(0, 51, 160);
  public static final Color LED_ESTOP_COLOR = Color.kRed;

  private LEDsConstants() {
    throw new IllegalAccessError();
  }
}
