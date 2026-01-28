package com.frc6324.robot2026.subsystems.drive;

import static com.frc6324.robot2026.subsystems.drive.DrivetrainConstants.ODOMETRY_PERIOD;
import static edu.wpi.first.units.Units.Seconds;

import com.frc6324.robot2026.generated.TunerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public final class DriveIOSim extends DriveIOCTRE {
  @SuppressWarnings("unchecked")
  private final MapleSimDriveBase simulation =
      new MapleSimDriveBase(
          getPigeon2(),
          getModules(),
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);

  private final Notifier notifier = new Notifier(simulation::update);

  public DriveIOSim() {
    super();

    registerTelemetry(state -> state.Pose = simulation.getSimulatedDriveTrainPose());

    Notifier.setHALThreadPriority(true, 90);

    notifier.setName("Simulation Thread");
    notifier.startPeriodic(ODOMETRY_PERIOD.in(Seconds));
  }

  @Override
  public void resetPose(Pose2d pose) {
    simulation.setSimulationWorldPose(pose);
    Timer.delay(0.05);

    super.resetPose(pose);
  }

  @Override
  public void updateInputs(DriveInputs inputs) {
    super.updateInputs(inputs);

    var arena = SimulatedArena.getInstance();
    Logger.recordOutput("FieldSimulation/Algae", arena.getGamePiecesArrayByType("Algae"));
    Logger.recordOutput("FieldSimulation/Coral", arena.getGamePiecesArrayByType("Coral"));
  }
}