package com.frc6324.robot2026.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import com.frc6324.lib.util.LoggedTracer;
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
    motorsIO.setSpinnerVelocity(RPM.of(450));
  }

  public void runKickerWheel() {
    motorsIO.setKickerVelocity(RPM.of(2400));
  }

  public void stopIndexerWheel() {
    motorsIO.stopSpinner();
  }

  public void stopKickerWheel() {
    motorsIO.stopKicker();
  }
}
