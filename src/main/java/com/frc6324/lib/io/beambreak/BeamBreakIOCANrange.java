package com.frc6324.lib.io.beambreak;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

public class BeamBreakIOCANrange implements BeamBreakIO {
  private final CANrange canrange;

  private final StatusSignal<Boolean> detection;

  public BeamBreakIOCANrange(int id, CANBus canBus, String name, CANrangeConfiguration config) {
    canrange = new CANrange(id, canBus);

    tryUntilOk(5, () -> canrange.getConfigurator().apply(config));

    detection = canrange.getIsDetected();
    detection.setUpdateFrequency(100);
    canrange.optimizeBusUtilization();
  }

  public BeamBreakIOCANrange(int id, CANBus canBus, CANrangeConfiguration config) {
    this(id, canBus, "CANrange [id " + id + " on bus " + canBus.getName() + "]", config);
  }

  @Override
  public void updateInputs(BeamBreakInputs inputs) {
    inputs.connected = canrange.isConnected() && detection.refresh().getStatus().isOK();

    inputs.broken = detection.getValue();
  }
}
