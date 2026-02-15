package com.frc6324.robot2026.subsystems.rollers;

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
    setDefaultCommand(idle());

    this.io = io;
    leaderDisconnectedAlert.set(false);
    followerDisconnectedAlert.set(false);
  }

  /**
   * Commands the rollers to coast out to save power when they aren't actively being used (e.g.
   * under the trench)
   */
  public void coastRollers() {
    io.coast();
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
  }

  /** Starts the intake's rollers. */
  public void spinRollers() {
    io.start();
  }

  /** Immediately stops the intake's rollers. */
  public void stopRollers() {
    io.stop();
  }
}
