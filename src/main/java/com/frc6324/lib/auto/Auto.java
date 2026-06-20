package com.frc6324.lib.auto;

import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.frc6324.lib.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.*;
import java.util.function.Supplier;

/**
 * A record containing all necessary information about an auto.
 *
 * <p>See {@link #Auto(String, Command)} or {@link #Auto(String, List, Supplier) Auto(String,
 * List<Trajectory<SwerveSample>>, Supplier)} for more specific constructors.
 *
 * @param name The name of the auto.
 * @param previewInfo Optional information about the auto path, if it is a trajectory-based auto.
 * @param commandSupplier A function that lazily creates the autonomous command.
 */
public record Auto(
    String name, Optional<Auto.TrajectoryInfo> previewInfo, Supplier<Command> commandSupplier) {
  /**
   * Creates a simple command-based auto.
   *
   * @param name The name of this auto.
   * @param cmd A function that returns the command to run.
   */
  public Auto(final String name, final Supplier<Command> cmd) {
    this(name, Optional.empty(), () -> cmd.get());
  }

  /**
   * Creates a trajectory-based auto.
   *
   * @param name The name of this auto.
   * @param trajectories The trajectories this path uses.
   * @param routineSupplier A function that creates the auto routine that contains the actual auto
   *     logic.
   */
  public Auto(
      final String name,
      final List<Trajectory<SwerveSample>> trajectories,
      final Supplier<AutoRoutine> routineSupplier) {
    this(
        name,
        Optional.of(new TrajectoryInfo(trajectories)),
        () -> {
          final AutoRoutine routine = routineSupplier.get();
          return routine.cmd().andThen(Commands.runOnce(routine::kill));
        });
  }

  /**
   * Builds and returns this auto command.
   *
   * @return The auto command.
   */
  public Command command() {
    return commandSupplier().get();
  }

  public static class TrajectoryInfo {
    /** The starting pose of the auto. This is always on the BLUE alliance. */
    public final Pose2d blueStartingPosition;

    /** The poses of each sample of each trajectory, on the BLUE alliance. */
    public final Pose2d[] blueTrajectoryPoses;

    /**
     * Creates a new trajectory preview object for the given trajectories.
     *
     * @param trajectories The trajectories that make up the auto.
     * @throws IllegalArgumentException If {@code trajectories} is empty, or every trajectory in
     *     {@code trajectories} is empty.
     */
    public TrajectoryInfo(final List<Trajectory<SwerveSample>> trajectories) {
      // If there are no trajectories, throw an exception.
      if (trajectories.isEmpty()) {
        throw new IllegalArgumentException(
            "Cannot build a trajectory auto with no trajectories in it.");
      }

      // Iterate through every trajectory to find the first known pose
      Optional<Pose2d> startPoseOpt = Optional.empty();
      for (final Trajectory<SwerveSample> traj : trajectories) {
        startPoseOpt = traj.getInitialPose(false);

        // If a starting pose has been found, stop
        if (startPoseOpt.isPresent()) {
          break;
        }
      }

      /** If every trajectory is empty, throw an exception. */
      if (startPoseOpt.isEmpty()) {
        throw new IllegalArgumentException(
            "Cannot build a trajectory auto consisting of empty trajectories.");
      }

      this.blueStartingPosition = startPoseOpt.get();

      // Get all poses from each trajectory
      final List<Pose2d> allPoses = new ArrayList<>();
      for (final Trajectory<SwerveSample> traj : trajectories) {
        Collections.addAll(allPoses, traj.getPoses());
      }

      this.blueTrajectoryPoses = allPoses.toArray(Pose2d[]::new);
    }

    public Pose2d getStartingPose() {
      return AllianceFlipUtil.apply(blueStartingPosition);
    }

    public Pose2d[] getTrajectoryPoses() {
      return AllianceFlipUtil.apply(blueTrajectoryPoses);
    }
  }
}
