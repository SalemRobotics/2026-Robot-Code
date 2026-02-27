package com.frc6324.lib.io.distance;

import static com.frc6324.lib.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;

public class DistanceSensorIOCANrange implements DistanceSensorIO {
  private final CANrange canrange;

  private final StatusSignal<Distance> detectedDistance;
  private final StatusSignal<Distance> detectedDistanceStddev;
  private final StatusSignal<Boolean> hasDetection;

  private final BaseStatusSignal[] signals;

  public DistanceSensorIOCANrange(int id, CANBus bus, CANrangeConfiguration config) {
    canrange = new CANrange(id, bus);

    tryUntilOk(5, () -> canrange.getConfigurator().apply(config, 0.25));

    detectedDistance = canrange.getDistance();
    detectedDistanceStddev = canrange.getDistanceStdDev();
    hasDetection = canrange.getIsDetected();
    signals = new BaseStatusSignal[] {detectedDistance, detectedDistanceStddev, hasDetection};

    BaseStatusSignal.setUpdateFrequencyForAll(100, signals);
    canrange.optimizeBusUtilization(0, 0.25);
  }

  @Override
  public boolean isActive() {
    return BaseStatusSignal.isAllGood(signals) && hasDetection.getValue();
  }

  @Override
  public void updateInputs(DistanceSensorInputs inputs) {
    inputs.connected = BaseStatusSignal.isAllGood(signals);

    inputs.detectedDistance = detectedDistance.getValue();
    inputs.distanceStddev = detectedDistanceStddev.getValue();
  }
}
