package com.frc6324.robot2026.subsystems.leds;

import static com.frc6324.robot2026.subsystems.leds.LEDsConstants.*;

import com.frc6324.lib.UninstantiableClass;
import com.frc6324.lib.util.LoggedTracer;
import com.frc6324.lib.util.VirtualSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;

public final class LEDs extends VirtualSubsystem {
  private final AddressableLED leds = new AddressableLED(LED_RIO_PWM_PORT);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_BUFFER_LENGTH);

  private final AddressableLEDBufferView fmsView =
      buffer.createView(LED_FMS_BUFFER_START, LED_BUFFER_LENGTH - 1);
  private final AddressableLEDBufferView dataView = buffer.createView(0, LED_FMS_BUFFER_START - 1);
  private final Debouncer browoutDebouncer = new Debouncer(1, DebounceType.kRising);

  public LEDs() {
    leds.setLength(LED_BUFFER_LENGTH);
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    final LEDPattern fmsPattern =
        DriverStation.isDSAttached()
            ? (DriverStation.isFMSAttached()
                ? LED_ALL_CONNECTED_PATTERN
                : LED_FMS_DISCONNECTED_PATTERN)
            : LED_DS_DISCONNECTED_PATTERN;
    fmsPattern.applyTo(fmsView);

    final boolean browningOut = browoutDebouncer.calculate(RobotController.isBrownedOut());

    final LEDPattern dataPattern;
    if (DriverStation.isEStopped()) {
      dataPattern = LED_ESTOP_PATTERN;
    } else if (browningOut) {
      dataPattern = LED_BROWNOUT_PATTERN;
    } else {
      if (LEDState.shooting) {
        dataPattern = LED_SHOOTING_PATTERN;
      } else if (LEDState.passing) {
        dataPattern = LED_PASSING_PATTERN;
      } else if (LEDState.outtaking) {
        dataPattern = LED_OUTTAKING_PATTERN;
      } else if (LEDState.intaking) {
        dataPattern = LED_INTAKING_PATTERN;
      } else {
        dataPattern = LED_DEFAULT_PATTERN;
      }
    }
    dataPattern.applyTo(dataView);

    leds.setData(buffer);
    LoggedTracer.record("LEDs periodic");
  }

  @UninstantiableClass
  public static final class LEDState {
    public static boolean passing = false;
    public static boolean shooting = false;
    public static boolean outtaking = false;
    public static boolean intaking = false;

    private LEDState() {
      throw new IllegalAccessError();
    }
  }
}
