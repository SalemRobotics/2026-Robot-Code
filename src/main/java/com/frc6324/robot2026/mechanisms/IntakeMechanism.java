package com.frc6324.robot2026.mechanisms;

import com.frc6324.lib.LazySingleton;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

@LazySingleton
public final class IntakeMechanism {
  private static IntakeMechanism instance = null;

  private final LoggedMechanism2d mechanism =
      new LoggedMechanism2d(Units.inchesToMeters(27), Units.inchesToMeters(22.5));
  private final LoggedMechanismLigament2d intake =
      mechanism.getRoot("Intake Root", 0, 0).append(new LoggedMechanismLigament2d("Intake", 0, 0));

  private IntakeMechanism() {
    throw new IllegalAccessError();
  }

  public static IntakeMechanism getInstance() {
    if (instance == null) {
      instance = new IntakeMechanism();
    }

    return instance;
  }

  public void log() {
    Logger.recordOutput("Intake/Mechanism", mechanism);
  }

  public void setExtension(Distance extensionMeters) {
    intake.setLength(extensionMeters);
  }
}
