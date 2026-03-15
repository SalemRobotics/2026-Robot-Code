package com.frc6324.robot2026.subsystems.leds;

import static com.frc6324.robot2026.subsystems.leds.LEDsConstants.*;

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
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public final class LEDs extends VirtualSubsystem {
  private final AddressableLED leds = new AddressableLED(LED_RIO_PWM_PORT);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_BUFFER_LENGTH);

  private final AddressableLEDBufferView fmsView =
      buffer.createView(LED_FMS_BUFFER_START, LED_BUFFER_LENGTH - 1);
  private final AddressableLEDBufferView dataView = buffer.createView(0, LED_FMS_BUFFER_START - 1);
  private final Debouncer browoutDebouncer = new Debouncer(1, DebounceType.kRising);
  @Setter private static LEDState state = LEDState.INACTIVE;

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
      dataPattern = state.pattern;
    }
    dataPattern.applyTo(dataView);

    leds.setData(buffer);
    LoggedTracer.record("LEDs periodic");
  }

  /**
   * An enum representing the current state of the robot's LEDs, used for other subsystems to set
   * their state.
   */
  @RequiredArgsConstructor
  public enum LEDState {
    /** LEDs are inactive, being either red (for an e-stop) or blue (default) */
    INACTIVE(LED_DEFAULT_PATTERN),
    INTAKING(LED_INTAKING_PATTERN),
    SHOOTING(LED_SHOOTING_PATTERN);

    public final LEDPattern pattern;
  }
}
