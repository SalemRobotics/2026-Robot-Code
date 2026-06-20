package com.frc6324.robot2026.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.io.IOLayer;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface ShooterIO extends IOLayer<ShooterIO.ShooterInputs> {
  default void coastDrum() {}

  default void setAcceleratorVelocity(AngularVelocity velocity) {}

  default void setDrumVelocity(AngularVelocity velocity, int slot) {}

  default void setHoodPosition(Angle position) {}

  default void stopAcceleratorMotor() {}

  @AutoLog
  public class ShooterInputs {
    public boolean hoodConnected = false;
    public boolean acceleratorConnected = false;
    public boolean[] drumMotorsConnected = {false, false, false, false};

    public Angle hoodPosition = Rotations.zero();
    public Voltage hoodMotorVoltage = Volts.zero();
    public Current hoodStatorCurrent = Amps.zero();

    public AngularVelocity acceleratorVelocity = RPM.zero();
    public Voltage acceleratorMotorVoltage = Volts.zero();
    public Current acceleratorStatorCurrent = Amps.zero();

    public AngularVelocity drumVelocity = RPM.zero();
    public double[] drumMotorVoltages = {0, 0, 0, 0};
    public double[] drumStatorCurrents = {0, 0, 0, 0};
  }
}
