package com.frc6324.robot2026.subsystems.drive;

import com.frc6324.lib.util.logging.LoggedTunableNumber;
import edu.wpi.first.math.controller.PIDController;

public class DrivePID {
  public final PIDController x;
  public final PIDController y;
  public final PIDController heading;

  private final LoggedTunableNumber translationP;
  private final LoggedTunableNumber translationI;
  private final LoggedTunableNumber translationD;
  private final LoggedTunableNumber headingP;
  private final LoggedTunableNumber headingI;
  private final LoggedTunableNumber headingD;

  public DrivePID(
      String prefix,
      double translationP,
      double translationI,
      double translationD,
      double headingP,
      double headingI,
      double headingD) {
    x = new PIDController(translationP, translationI, translationD);
    y = new PIDController(translationP, translationI, translationD);
    heading = new PIDController(headingP, headingI, headingD);

    this.translationP =
        new LoggedTunableNumber("DrivePID/" + prefix + "/Translation/kP", translationP);
    this.translationI =
        new LoggedTunableNumber("DrivePID/" + prefix + "/Translation/kI", translationI);
    this.translationD =
        new LoggedTunableNumber("DrivePID/" + prefix + "/Translation/kD", translationD);
    this.headingP = new LoggedTunableNumber("DrivePID/" + prefix + "/Heading/kP", headingP);
    this.headingI = new LoggedTunableNumber("DrivePID/" + prefix + "/Heading/kI", headingI);
    this.headingD = new LoggedTunableNumber("DrivePID/" + prefix + "/Heading/kD", headingD);
  }

  public void update() {
    // Check/update translation PID
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          final double p = translationP.get();
          final double i = translationI.get();
          final double d = translationD.get();

          x.setPID(p, i, d);
          y.setPID(p, i, d);
        },
        translationP,
        translationI,
        translationD);

    // Check/update heading PID
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> heading.setPID(headingP.get(), headingI.get(), headingD.get()),
        headingP,
        headingI,
        headingD);
  }
}
