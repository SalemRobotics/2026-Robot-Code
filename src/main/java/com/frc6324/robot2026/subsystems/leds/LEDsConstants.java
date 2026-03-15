package com.frc6324.robot2026.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.frc6324.lib.UninstantiableClass;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

@UninstantiableClass
public final class LEDsConstants {
  public static final int LED_RIO_PWM_PORT = 0;
  public static final int LED_BUFFER_LENGTH = 15;

  public static final int LED_FMS_BUFFER_START = 13;

  public static final Color LED_DEFAULT_COLOR = new Color(0, 51, 160);
  public static final LEDPattern LED_DEFAULT_PATTERN = LEDPattern.solid(LED_DEFAULT_COLOR);

  public static final LEDPattern LED_ALL_CONNECTED_PATTERN = LEDPattern.solid(Color.kGreen);
  public static final LEDPattern LED_DS_DISCONNECTED_PATTERN = LEDPattern.solid(Color.kRed);
  public static final LEDPattern LED_FMS_DISCONNECTED_PATTERN = LEDPattern.solid(Color.kPurple);
  public static final LEDPattern LED_ESTOP_PATTERN = LEDPattern.solid(Color.kRed);
  public static final LEDPattern LED_BROWNOUT_PATTERN =
      LEDPattern.solid(Color.kDarkOrange).blink(Seconds.of(2));
  public static final LEDPattern LED_INTAKING_PATTERN =
      LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.5), Seconds.of(0.2));
  public static final LEDPattern LED_SHOOTING_PATTERN =
      LEDPattern.gradient(GradientType.kContinuous, Color.kDeepPink, Color.kCyan)
          .scrollAtRelativeSpeed(Percent.per(Second).of(25));

  private LEDsConstants() {
    throw new IllegalAccessError();
  }
}
