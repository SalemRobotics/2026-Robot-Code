package com.frc6324.lib.auto;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.frc6324.lib.util.AllianceSide;
import com.frc6324.lib.util.DrivePID;
import com.frc6324.lib.util.Drivetrain;
import com.frc6324.robot2026.RobotState;
import com.frc6324.robot2026.generated.ChoreoTraj;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import lombok.Getter;

public class AutoBuilder<S extends AutoBuilder.SuperstructureBase> {
  @Getter private final AutoFactory factory;
  @Getter private final S superstructure;

  private final Drivetrain drivetrain;
  private final Map<String, Command> eventBindings = new HashMap<>();

  public AutoBuilder(RobotState robotState, Drivetrain drivetrain, S superstructure) {
    this.factory =
        new AutoFactory(
            robotState::getPose,
            robotState::setPose,
            (SwerveSample sample) -> {},
            true,
            drivetrain);
    this.superstructure = superstructure;
    this.drivetrain = drivetrain;

    superstructure.registerEventBindings(eventBindings);
  }

  private static Trajectory<SwerveSample> applyMirror(
      AllianceSide side, Trajectory<SwerveSample> trajectory) {
    if (side == AllianceSide.Left) {
      return trajectory;
    }

    return trajectory.mirrorY();
  }

  public TrajectoryFollower follow(Trajectory<SwerveSample> trajectory) {
    final DrivePID autoPID = drivetrain.getAutoDrivePID();

    return new TrajectoryFollower(
        autoPID.x, autoPID.y, autoPID.heading, trajectory, drivetrain, eventBindings);
  }

  public Optional<List<Trajectory<SwerveSample>>> load(
      AllianceSide side, ChoreoTraj... trajectories) {
    final List<Trajectory<SwerveSample>> loaded = new ArrayList<>(trajectories.length);
    boolean hasMissing = false;

    for (final var trajectory : trajectories) {
      final String name = trajectory.name();
      final Optional<Trajectory<SwerveSample>> trajOpt = Choreo.loadTrajectory(name);
      final Trajectory<SwerveSample> traj = trajOpt.orElse(null);

      if (traj == null) {
        DriverStation.reportError("Failed to load auto path \"" + name + "\"", false);
        hasMissing = true;
        continue;
      }

      loaded.add(applyMirror(side, traj));
    }

    return hasMissing ? Optional.empty() : Optional.of(loaded);
  }

  public record LoadedTrajectories(List<Trajectory<SwerveSample>> trajectories, int missing) {}

  public interface SuperstructureBase {
    void registerEventBindings(Map<String, Command> bindings);
  }
}
