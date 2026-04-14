package com.frc6324.robot2026.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface ShooterIO {
  default void coastDrum() {}

  default void setAcceleratorVelocity(AngularVelocity velocity) {}

  default void setDrumVelocity(AngularVelocity velocity, int slot) {}

  default void setHoodPosition(Angle position) {}

  default void stopAcceleratorMotor() {}

  void updateInputs(ShooterInputs inputs);

  @AutoLog
  public class ShooterInputs {
    public boolean hoodConnected = false;
    public boolean acceleratorConnected = false;
    public boolean[] drumMotorsConnected = new boolean[] {false, false, false, false};

    public Angle hoodPosition = Rotations.zero();
    public Voltage hoodMotorVoltage = Volts.zero();
    public Current hoodStatorCurrent = Amps.zero();

    public AngularVelocity acceleratorVelocity = RPM.zero();
    public Voltage acceleratorMotorVoltage = Volts.zero();
    public Current acceleratorStatorCurrent = Amps.zero();

    public AngularVelocity drumVelocity = RPM.zero();
    public double[] drumMotorVoltages = new double[] {0, 0, 0, 0};
    public double[] drumStatorCurrents = new double[] {0, 0, 0, 0};
  }
}
