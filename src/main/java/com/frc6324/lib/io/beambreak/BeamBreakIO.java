package com.frc6324.lib.io.beambreak;

import org.littletonrobotics.junction.AutoLog;

/**
 * An I/O layer over a beam break sensor.
 *
 * <p>A beam break sensor is a sensor that emits a beam from itself, and has some mechanism that
 * detects when the beam is being broken by an object.
 *
 * <p>This I/O layer also works for digital Time of Flight (ToF) sensors that report if the detected
 * time of flight is below a certain threshold.
 */
public interface BeamBreakIO {
  /**
   * Updates a set of loggable inputs for this beam break.
   *
   * @param inputs The inputs to modifiy.
   */
  void updateInputs(BeamBreakInputs inputs);

  /** Inputs for a beam break sensor. */
  @AutoLog
  class BeamBreakInputs {
    /**
     * Whether the sensor is connected.
     *
     * <p>Note that this is not always accurate, especially when using I/O implementations that have
     * no method of reporting connection values (e.g. {@link BeamBreakIODIO}, {@link
     * BeamBreakIOFilteredDIO})
     */
    public boolean connected = false;

    /** Whether the sensor is detecting a break in the beam. */
    public boolean broken = false;
  }
}
