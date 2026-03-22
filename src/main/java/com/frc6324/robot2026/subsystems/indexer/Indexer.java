package com.frc6324.robot2026.subsystems.indexer;

import static com.frc6324.robot2026.subsystems.indexer.IndexerConstants.*;
import static edu.wpi.first.units.Units.RPM;

import com.frc6324.lib.util.LoggedTracer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Indexer extends SubsystemBase {
  private final IndexerIO motorsIO;
  private final IndexerInputsAutoLogged motorInputs = new IndexerInputsAutoLogged();

  public Indexer(IndexerIO indexer) {
    motorsIO = indexer;
  }

  public boolean isFull() {
    return false;
  }

  @Override
  public void periodic() {
    motorsIO.updateInputs(motorInputs);
    Logger.processInputs("Indexer", motorInputs);
    LoggedTracer.record("Indexer periodic");
  }

  public void runIndexerWheel() {
    final AngularVelocity addition = INDEXER_SPINNER_SHAKE_ADDITION.times(Math.random());
    final AngularVelocity velocity = INDEXER_SPINNER_VELOCITY.plus(addition);

    Logger.recordOutput("Indexer/SpinnerVelocitySetpoint", velocity);
    motorsIO.setSpinnerVelocity(velocity);
  }

  public void runKickerWheel() {
    motorsIO.setKickerVelocity(INDEXER_KICKER_VELOCITY);
  }

  public void stopIndexerWheel() {
    Logger.recordOutput("Indexer/SpinnerVelocitySetpoint", RPM.zero());
    motorsIO.stopSpinner();
  }

  public void stopKickerWheel() {
    motorsIO.stopKicker();
  }
}
