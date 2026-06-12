package com.frc6324.robot2026.commands.autos;

import static com.frc6324.robot2026.generated.ChoreoTraj.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.lib.util.AllianceSide;
import com.frc6324.robot2026.commands.autos.Auto.LoadedTrajectories;
import com.frc6324.robot2026.generated.ChoreoTraj;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@UninstantiableClass
public final class NeutralZoneAutos {
  private NeutralZoneAutos() {
    throw new IllegalAccessError();
  }

  public static void addToChooser(LoggedDashboardChooser<Auto> chooser, Auto.Builder builder) {
    regularDoublePass(AllianceSide.Left, builder)
        .ifPresent(auto -> chooser.addOption("Left Double Pass", auto));

    regularDoublePass(AllianceSide.Right, builder)
        .ifPresent(auto -> chooser.addOption("Right Double Pass", auto));
  }

  /**
   * Creates a regular 'double pass' auto that enters the neutral zone, intakes, returns over the
   * bump and shoots twice.
   *
   * @param side The side of the alliance the auto will run on.
   * @param builder The auto builder to use.
   * @return The double pass auto if it loaded successfully.
   */
  public static Optional<Auto> regularDoublePass(AllianceSide side, Auto.Builder builder) {
    final String name = side + "DoublePass";
    final ChoreoTraj[] trajectories = {TrenchStartPass, HubShotNZPass};

    final LoadedTrajectories loaded = builder.load(side, trajectories);
    if (loaded.missing() > 0) {
      // If the missing trajectory count is greater than 0, the errors have already
      // been reported,
      // so return an empty optional
      return Optional.empty();
    }

    final List<Trajectory<SwerveSample>> loadedTrajectories = loaded.trajectories();

    return Optional.of(
        Auto.followTrajectory(
            name,
            loadedTrajectories,
            () -> {
              final AutoRoutine routine = builder.factory.newRoutine(name);

              final AutoTrajectory first = routine.trajectory(loadedTrajectories.get(0));

              final TrajectoryFollower firstPass = builder.follow(loadedTrajectories.get(0));
              final TrajectoryFollower secondPass = builder.follow(loadedTrajectories.get(1));

              // NOTE: proxies are used here for re-use of commands because Commands V2
              // doesn't
              // allow you to manually schedule commands in a composition or

              routine.active().onTrue(firstPass.beforeStarting(first.resetOdometry()));
              routine
                  .observe(firstPass.done())
                  .onTrue(Commands.sequence(builder.shootCommand(3), secondPass.asProxy()));
              routine.observe(secondPass.done()).onTrue(builder.shootCommand(5).asProxy());

              return routine;
            }));
  }
}
