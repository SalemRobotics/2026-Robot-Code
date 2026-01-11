package com.frc6324.robot2026.subsystems.drive;

import static com.frc6324.robot2026.subsystems.drive.DrivetrainConstants.ODOMETRY_PERIOD;

import com.frc6324.lib.DeltaTimeCalculator;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

public final class DriveIOSim extends DriveIOCTRE {
  private final Notifier notifier;
  private final DeltaTimeCalculator deltaCalculator = new DeltaTimeCalculator();

  // TODO: update to using maple-sim like the template once it gets updated

  public DriveIOSim() {
    notifier =
        new Notifier(
            () -> {
              var delta = deltaCalculator.get();

              updateSimState(delta, RobotController.getBatteryVoltage());
            });

    notifier.setName("Simulation Thread");
    notifier.startPeriodic(ODOMETRY_PERIOD);
  }
}
