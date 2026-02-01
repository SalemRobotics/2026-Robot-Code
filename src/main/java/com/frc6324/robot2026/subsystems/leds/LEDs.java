package com.frc6324.robot2026.subsystems.leds;

import static com.frc6324.robot2026.subsystems.leds.LEDsConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;

public final class LEDs extends SubsystemBase {
  private final AddressableLED leds = new AddressableLED(LED_RIO_PWM_PORT);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_BUFFER_LENGTH);
  @Setter private LEDState currentState = LEDState.INACTIVE;

  public LEDs() {
    leds.setLength(LED_BUFFER_LENGTH);
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    if (DriverStation.isEStopped()) {
      LED_ESTOP_PATTERN.applyTo(buffer);
    } else if (RobotController.isBrownedOut()) {
      LED_BROWNOUT_PATTERN.applyTo(buffer);
    } else switch (currentState) {
      case INACTIVE -> LED_DEFAULT_PATTERN.applyTo(buffer);
    }
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
