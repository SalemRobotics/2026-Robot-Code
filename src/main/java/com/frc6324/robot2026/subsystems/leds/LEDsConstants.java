package com.frc6324.robot2026.subsystems.leds;

import static edu.wpi.first.units.Units.Seconds;

import com.frc6324.lib.UninstantiableClass;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

@UninstantiableClass
public final class LEDsConstants {
  public static final int LED_RIO_PWM_PORT = 0;
  public static final int LED_BUFFER_LENGTH = 8;

  public static final Color LED_DEFAULT_COLOR = new Color(0, 51, 160);
  public static final LEDPattern LED_DEFAULT_PATTERN = LEDPattern.solid(LED_DEFAULT_COLOR);

  public static final Color LED_ESTOP_COLOR = Color.kRed;
  public static final LEDPattern LED_ESTOP_PATTERN = LEDPattern.solid(LED_ESTOP_COLOR);

  public static final Color LED_BROWNOUT_COLOR = Color.kDarkOrange;
  public static final LEDPattern LED_BROWNOUT_PATTERN =
      LEDPattern.solid(LED_BROWNOUT_COLOR).blink(Seconds.of(2));

  private LEDsConstants() {
    throw new IllegalAccessError();
  }
}
