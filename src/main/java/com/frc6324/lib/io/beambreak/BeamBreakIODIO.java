package com.frc6324.lib.io.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;

/** A beam break sensor on one of the RoboRIO's Digital I/O (DIO) ports. */
public class BeamBreakIODIO implements BeamBreakIO {
  /** The DIO sensor. */
  protected final DigitalInput sensor;

  /** Whether the sensor is inverted. */
  private final boolean inverted;

  /**
   * Creates a new beam break sensor using a real digital I/O (DIO) device.
   *
   * @param dioDevice The Digital I/O device to read.
   * @param inverted Whether the sensor is inverted.
   */
  public BeamBreakIODIO(DigitalInput dioDevice, boolean inverted) {
    this.sensor = dioDevice;
    this.inverted = inverted;
  }

  /**
   * Creates a new beam break sensor using a real digital I/O (DIO) device.
   *
   * @param dioDevice The Digital I/O device to read.
   */
  public BeamBreakIODIO(DigitalInput dioDevice) {
    this(dioDevice, false);
  }

  /**
   * Creates a new beam break sensor using a real digital I/O (DIO) device.
   *
   * @param dioPort The DIO port the device is on. Note that if the device is disconnected, the
   *     behavior of this I/O layer is not guaranteed.
   * @param inverted Whether the sensor is inverted.
   */
  public BeamBreakIODIO(int dioPort, boolean inverted) {
    this(new DigitalInput(dioPort), inverted);
  }

  /**
   * Creates a new beam break sensor using a real digital I/O (DIO) device.
   *
   * @param dioPort The DIO port the device is on. Note that if the device is disconnected, the
   *     behavior of this I/O layer is not guaranteed.
   */
  public BeamBreakIODIO(int dioPort) {
    this(new DigitalInput(dioPort), false);
  }

  @Override
  public void updateInputs(BeamBreakInputs inputs) {
    inputs.connected = true;
    inputs.broken = Boolean.logicalXor(inverted, sensor.get());
  }
}
