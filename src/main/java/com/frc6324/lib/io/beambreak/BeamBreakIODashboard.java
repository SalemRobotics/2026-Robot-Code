package com.frc6324.lib.io.beambreak;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * A beam break sensor that uses a NetworkTables entry to determine the value reported by the sensor
 * according to a user.
 */
public final class BeamBreakIODashboard implements BeamBreakIO {
  private final LoggedNetworkBoolean networkBoolean;

  /**
   * Creates a new beam break sensor with a backing NetworkTables boolean value.
   *
   * <p>This implementation is meant to be used during non-replay simulation.
   *
   * @param networkBoolean The {@link LoggedNetworkBoolean} that represents the sensor value.
   */
  public BeamBreakIODashboard(LoggedNetworkBoolean networkBoolean) {
    this.networkBoolean = networkBoolean;
  }

  /**
   * Creates a new beam break sensor with a backing NetworkTables boolean value.
   *
   * <p>This implementation is meant to be used during non-replay simulation.
   *
   * @param key The path to the boolean value in NetworkTables.
   */
  public BeamBreakIODashboard(String key) {
    this(new LoggedNetworkBoolean(key));
  }

  /**
   * Creates a new beam break sensor with a backing NetworkTables boolean value.
   *
   * <p>This implementation is meant to be used during non-replay simulation.
   *
   * @param key The path to the boolean value in NetworkTables.
   * @param defaultValue The default value put onto the network.
   */
  public BeamBreakIODashboard(String key, boolean defaultValue) {
    this(new LoggedNetworkBoolean(key, defaultValue));
  }

  @Override
  public void updateInputs(BeamBreakInputs inputs) {
    inputs.connected = true;
    inputs.broken = networkBoolean.get();
  }
}
