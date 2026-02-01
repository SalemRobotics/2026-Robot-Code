package com.frc6324.robot2026.subsystems.leds;

import static com.frc6324.robot2026.subsystems.leds.LEDsConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public final class LEDs extends SubsystemBase {
  private final AddressableLED leds = new AddressableLED(LED_RIO_PWM_PORT);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_BUFFER_LENGTH);
  private Color currentColor = LED_DEFAULT_COLOR;
  @Setter private LEDState currentState = LEDState.INACTIVE;

  public LEDs() {
    leds.setLength(LED_BUFFER_LENGTH);
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case INACTIVE -> {
        if (DriverStation.isEStopped()) {
          solid(LED_ESTOP_COLOR);
        } else {
          solid(LED_DEFAULT_COLOR);
        }
      }
    }

    Logger.recordOutput("LEDs/CurrentColor", currentColor.toHexString());
  }

  private void solid(Color color) {
    for (int i = 0; i < LED_BUFFER_LENGTH; i++) {
      buffer.setLED(i, color);
    }

    currentColor = color;
  }

  /**
   * An enum representing the current state of the robot's LEDs, used for other subsystems to set
   * their state.
   */
  public enum LEDState {
    /** LEDs are inactive, being either red (for an e-stop) or blue (default) */
    INACTIVE,
  }
}
