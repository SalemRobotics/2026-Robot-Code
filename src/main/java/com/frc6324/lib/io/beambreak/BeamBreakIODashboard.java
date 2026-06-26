package com.frc6324.lib.io.beambreak;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A beam break sensor that uses a NetworkTables entry to determine the value reported by the sensor
 * according to a user.
 */
public final class BeamBreakIODashboard implements BeamBreakIO {
  private final BooleanSubscriber subscriber;

  /**
   * Creates a new beam break sensor with a backing NetworkTables boolean value.
   *
   * <p>This implementation is meant to be used during non-replay simulation.
   *
   * @param key The path to the boolean value in NetworkTables.
   * @param defaultValue The default value put onto the network.
   */
  public BeamBreakIODashboard(String key, boolean defaultValue) {
    final BooleanTopic topic = NetworkTableInstance
      .getDefault()
      .getBooleanTopic(key);
    
    this.subscriber = topic.subscribe(defaultValue);
  }

  @Override
  public void updateInputs(BeamBreakInputs inputs) {
    inputs.connected = true;

    inputs.broken = subscriber.get();
  }
}
