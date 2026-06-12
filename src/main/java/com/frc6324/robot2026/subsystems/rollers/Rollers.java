package com.frc6324.robot2026.subsystems.rollers;

import com.frc6324.lib.util.logging.LoggedTracer;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Rollers extends SubsystemBase {
  private final RollerIO io;
  private final RollerInputsAutoLogged inputs = new RollerInputsAutoLogged();
  private final Debouncer leaderDiconnectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerDisconnectedDebouncer = new Debouncer(0.5);
  private final Alert leaderDisconnectedAlert =
      new Alert("Lead roller motor is disconnected!", AlertType.kError);
  private final Alert followerDisconnectedAlert =
      new Alert("Follower roller motor is disconnected!", AlertType.kError);

  /**
   * Creates a new roller subsystem with the given I/O implementation.
   *
   * @param io The implementation to use for the rollers.
   */
  public Rollers(RollerIO io) {
    setName("Intake Rollers");

    this.io = io;
    leaderDisconnectedAlert.set(false);
    followerDisconnectedAlert.set(false);
  }

  @Override
  public void periodic() {
    // Update & log inputs
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Rollers", inputs);

    // Update the alerts using the debounced connection values
    final boolean leaderDisconnected =
        leaderDiconnectedDebouncer.calculate(!inputs.leaderConnected);
    final boolean followerDisconnected =
        followerDisconnectedDebouncer.calculate(!inputs.followerConnected);
    leaderDisconnectedAlert.set(leaderDisconnected);
    followerDisconnectedAlert.set(followerDisconnected);

    LoggedTracer.record("Periodic/Rollers periodic");
  }

  /** Starts the intake's rollers. */
  public void spinRollers() {
    io.start();
  }

  /** Immediately stops the intake's rollers. */
  public void stopRollers() {
    io.stop();
  }

  public void outtake() {
    io.outtake();
  }
}
