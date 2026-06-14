package com.frc6324.lib.util.logging;

import edu.wpi.first.math.controller.PIDController;

/**
 * A PIDController with tunable gains that can be adjusted from the dashboard.
 *
 * <p>This controller extends WPILib's PIDController to add runtime tunability. PID gains (kP, kI,
 * kD) can be adjusted through NetworkTables when tuning mode is enabled.
 *
 * <p>Call {@link #updatePID()} periodically to check for and apply updated values from the
 * dashboard.
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * LoggedTunablePID controller =
 *     new LoggedTunablePID("Arm/PID", 2.0, 0.0, 0.1);
 *
 * // In periodic():
 * controller.updatePID(); // Apply any changes from dashboard
 * double output = controller.calculate(position, setpoint);
 * }</pre>
 */
public class LoggedTunablePID extends PIDController {
  private final LoggedTunableNumber p, i, d;

  /**
   * Constructs a tunable PID controller with default period.
   *
   * @param name The logging key prefix for tunable values
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   */
  public LoggedTunablePID(String name, double kP, double kI, double kD) {
    this(name, kP, kI, kD, 0.02);
  }

  /**
   * Constructs a tunable profiled PID controller with specified period.
   *
   * @param name The logging key prefix for tunable values
   * @param p Proportional gain
   * @param i Integral gain
   * @param d Derivative gain
   * @param period Loop period in seconds
   */
  public LoggedTunablePID(String name, double kP, double kI, double kD, double period) {
    super(kP, kI, kD);

    // Tunable numbers for PID and motion gain constants
    this.p = new LoggedTunableNumber(name + "/kP", kP);
    this.i = new LoggedTunableNumber(name + "/kI", kI);
    this.d = new LoggedTunableNumber(name + "/kD", kD);
  }

  /** Updates PID from tunable values if changed. */
  public void updatePID() {
    // If changed, update controller constants from Tuneable Numbers
    if (p.hasChanged(hashCode()) || i.hasChanged(hashCode()) || d.hasChanged(hashCode())) {
      this.setPID(p.get(), i.get(), d.get());
    }
  }
}
