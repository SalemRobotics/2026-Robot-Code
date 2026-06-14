package com.frc6324.robot2026.commands.autos;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.frc6324.lib.util.AllianceSide;
import com.frc6324.lib.util.FieldConstants;
import com.frc6324.robot2026.RobotState;
import com.frc6324.robot2026.commands.ShooterCommands.ShootIntoHubCommand;
import com.frc6324.robot2026.generated.ChoreoTraj;
import com.frc6324.robot2026.subsystems.drive.Drive;
import com.frc6324.robot2026.subsystems.indexer.Indexer;
import com.frc6324.robot2026.subsystems.intake.Intake;
import com.frc6324.robot2026.subsystems.rollers.Rollers;
import com.frc6324.robot2026.subsystems.shooter.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

public record Auto(
    String name,
    List<Pose2d> previewPoses,
    Pose2d startingPose,
    Supplier<Command> commandSupplier) {

  public Command command() {
    return commandSupplier.get();
  }

  public static Auto doNothing() {
    return new Auto("Do Nothing Auto", List.of(), Pose2d.kZero, Commands::none);
  }

  public static Auto followTrajectory(
      String name,
      List<Trajectory<SwerveSample>> trajectories,
      Supplier<AutoRoutine> routineSupplier) {

    final List<Pose2d> previewPoses = new ArrayList<>();
    for (Trajectory<SwerveSample> trajectory : trajectories) {
      previewPoses.addAll(List.of(trajectory.getPoses()));
    }

    final Pose2d start =
        trajectories.isEmpty()
            ? Pose2d.kZero
            : trajectories.get(0).getInitialPose(false).orElse(Pose2d.kZero);

    return new Auto(
        name,
        previewPoses,
        start,
        () -> {
          final AutoRoutine routine = routineSupplier.get();

          return Commands.sequence(routine.cmd(), Commands.runOnce(routine::kill));
        });
  }

  public record LoadedTrajectories(List<Trajectory<SwerveSample>> trajectories, int missing) {}

  public static class Builder {
    public final AutoFactory factory;
    final RobotState robotState;
    final Drive drive;
    final Indexer indexer;
    final Intake intake;
    final Rollers rollers;
    final Shooter shooter;
    final Map<String, Command> eventBindings = new HashMap<>();

    public Builder(
        RobotState robotState,
        Drive drive,
        Indexer indexer,
        Intake intake,
        Rollers rollers,
        Shooter shooter) {
      this.robotState = robotState;
      this.drive = drive;
      this.indexer = indexer;
      this.intake = intake;
      this.rollers = rollers;
      this.shooter = shooter;

      factory =
          new AutoFactory(
              robotState::getPose, robotState::setPose, (SwerveSample sample) -> {}, true, drive);

      CommandScheduler.getInstance().schedule(factory.warmupCmd());
    }

    private static Trajectory<SwerveSample> applyMirror(
        AllianceSide side, Trajectory<SwerveSample> trajectory) {
      if (side == AllianceSide.Left) {
        return trajectory;
      }

      final List<SwerveSample> samples = trajectory.samples();
      final List<SwerveSample> mirroredSamples = new ArrayList<>();

      for (final SwerveSample sample : samples) {
        final double[] forcesY = sample.moduleForcesY();
        final double[] mirroredForcesY = new double[forcesY.length];

        for (int i = 0; i < forcesY.length; i++) {
          mirroredForcesY[i] = forcesY[i];
        }

        mirroredSamples.add(
            new SwerveSample(
                sample.t,
                sample.x,
                FieldConstants.FIELD_WIDTH - sample.y,
                -sample.heading,
                sample.vx,
                -sample.vy,
                -sample.omega,
                sample.ax,
                -sample.ay,
                -sample.alpha,
                sample.moduleForcesX(),
                mirroredForcesY));
      }

      return new Trajectory<>(
          trajectory.name(), mirroredSamples, trajectory.splits(), trajectory.events());
    }

    public TrajectoryFollower follow(Trajectory<SwerveSample> trajectory) {
      return new TrajectoryFollower(
          drive.autoPID.x,
          drive.autoPID.y,
          drive.autoPID.heading,
          trajectory,
          drive,
          eventBindings);
    }

    public LoadedTrajectories load(AllianceSide side, ChoreoTraj... trajectories) {
      final List<Trajectory<SwerveSample>> loaded = new ArrayList<>(trajectories.length);
      int missing = 0;

      for (final var trajectory : trajectories) {
        final String name = trajectory.name();
        final Optional<Trajectory<SwerveSample>> trajOpt = Choreo.loadTrajectory(name);
        final Trajectory<SwerveSample> traj = trajOpt.orElse(null);

        if (traj == null) {
          DriverStation.reportError("Failed to load auto path \"" + name + "\"", false);
          missing++;
          continue;
        }

        loaded.add(applyMirror(side, traj));
      }

      return new LoadedTrajectories(loaded, missing);
    }

    public Command shootCommand() {
      return new ShootIntoHubCommand(
              robotState, drive, indexer, intake, rollers, shooter, "AutoShootIntoHub")
          .until(shooter::noFuelSeen);
    }

    public Command shootCommand(double timeout) {
      return shootCommand().withTimeout(timeout);
    }
  }
}
