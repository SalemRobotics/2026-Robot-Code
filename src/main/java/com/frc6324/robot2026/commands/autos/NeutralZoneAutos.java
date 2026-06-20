package com.frc6324.robot2026.commands.autos;

import static com.frc6324.robot2026.generated.ChoreoTraj.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.frc6324.lib.auto.Auto;
import com.frc6324.lib.auto.AutoBuilder;
import com.frc6324.lib.auto.TrajectoryFollower;
import com.frc6324.lib.util.AllianceSide;
import com.frc6324.robot2026.generated.ChoreoTraj;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Optional;
import lombok.experimental.UtilityClass;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@UtilityClass
public final class NeutralZoneAutos {
  public static void addToChooser(
      LoggedDashboardChooser<Auto> chooser, AutoBuilder<Superstructure> builder) {
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
  public static Optional<Auto> regularDoublePass(
      AllianceSide side, AutoBuilder<Superstructure> builder) {
    final String name = side + "DoublePass";
    final ChoreoTraj[] trajectories = {TrenchStartPass, HubShotNZPass};

    final Optional<List<Trajectory<SwerveSample>>> loaded = builder.load(side, trajectories);
    if (loaded.isEmpty()) {
      return Optional.empty();
    }

    final List<Trajectory<SwerveSample>> loadedTrajectories = loaded.get();

    return Optional.of(
        new Auto(
            name,
            loadedTrajectories,
            () -> {
              final AutoRoutine routine = builder.getFactory().newRoutine(name);
              final Superstructure superstructure = builder.getSuperstructure();

              final AutoTrajectory first = routine.trajectory(loadedTrajectories.get(0));

              final TrajectoryFollower firstPass = builder.follow(loadedTrajectories.get(0));
              final TrajectoryFollower secondPass = builder.follow(loadedTrajectories.get(1));

              routine.active().onTrue(Commands.sequence(first.resetOdometry(), firstPass));

              routine
                  .observe(firstPass.done())
                  .onTrue(Commands.sequence(superstructure.shootCommand(3), secondPass.asProxy()));
              routine.observe(secondPass.done()).onTrue(superstructure.shootCommand(5).asProxy());

              return routine;
            }));
  }
}
