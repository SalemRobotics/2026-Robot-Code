package com.frc6324.robot2026.commands.autos;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.frc6324.lib.util.AllianceFlipUtil;
import com.frc6324.lib.util.logging.LoggedTrigger;
import com.frc6324.lib.util.logging.LoggedTunableNumber;
import com.frc6324.robot2026.RobotState;
import com.frc6324.robot2026.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public final class TrajectoryFollower extends Command {
  enum State {
    Following,
    Recovering
  }

  private static final LoggedTunableNumber PAUSE_THRESHOLD =
      new LoggedTunableNumber("Auto/FollowerCommand/PauseThreshold", 0.2);
  private static final LoggedTunableNumber RESUME_THRESHOLD =
      new LoggedTunableNumber("Auto/FollowerCommand/ResumeThreshold", 0.1);
  private static final LoggedTunableNumber PAUSE_DEBOUNCE =
      new LoggedTunableNumber("Auto/FollowerCommand/DebounceTime", 0.2);

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController headingController;

  private final Drive drive;
  private final RobotState robotState = RobotState.getInstance();
  private final Map<String, Command> eventCommands;
  private final Trajectory<SwerveSample> trajectory;

  private State state;
  private Debouncer pauseDebouncer;
  private boolean finished = false;

  // Time tracking info
  private double followingTime;
  private double lastTimestamp;

  // Event handling
  private final List<EventMarker> events;
  private int nextEventIndex = 0;

  public TrajectoryFollower(
      PIDController xController,
      PIDController yController,
      PIDController headingController,
      Trajectory<SwerveSample> trajectory,
      Drive drive,
      Map<String, Command> eventCommands) {
    this.xController = xController;
    this.yController = yController;
    this.headingController = headingController;
    this.headingController.enableContinuousInput(-Math.PI, Math.PI);

    this.drive = drive;
    this.events =
        trajectory.events().stream().sorted(Comparator.comparingDouble(m -> m.timestamp)).toList();
    this.eventCommands = eventCommands;
    this.trajectory = trajectory;

    addRequirements(drive);
  }

  private void triggerEvents() {
    while (nextEventIndex < events.size()) {
      final EventMarker next = events.get(nextEventIndex);

      if (followingTime >= next.timestamp) {
        final String name = next.event;
        final Command cmd = eventCommands.get(name);

        if (cmd != null) {
          CommandScheduler.getInstance().schedule(cmd);
        } else {
          DriverStation.reportError(
              "Command "
                  + name
                  + " referenced in trajectory "
                  + trajectory.name()
                  + " does not exist.",
              false);
        }

        nextEventIndex++;
      } else {
        break;
      }
    }
  }

  public LoggedTrigger done() {
    return new LoggedTrigger("Auto/FollowerFor " + trajectory.name(), () -> finished);
  }

  private void updateState(double translationalError) {
    switch (state) {
      case Following -> {
        boolean overThreshold =
            pauseDebouncer.calculate(translationalError > PAUSE_THRESHOLD.get());
        if (overThreshold) {
          state = State.Recovering;
          // Reset PID integrators to avoid windup from accumulated error.
          xController.reset();
          yController.reset();
          headingController.reset();
        }
      }
      case Recovering -> {
        if (translationalError < RESUME_THRESHOLD.get()) {
          state = State.Following;
          // Reset debouncer so momentary spikes after resuming don't re-trigger.
          pauseDebouncer = new Debouncer(PAUSE_DEBOUNCE.get());
          // Reset PID integrators for clean tracking restart.
          xController.reset();
          yController.reset();
          headingController.reset();
        }
      }
    }
  }

  @Override
  public void initialize() {
    state = State.Following;
    nextEventIndex = 0;

    followingTime = 0;
    lastTimestamp = Timer.getFPGATimestamp();
    finished = false;

    xController.reset();
    yController.reset();
    headingController.reset();

    pauseDebouncer = new Debouncer(PAUSE_DEBOUNCE.get());

    Logger.recordOutput("Auto/FollowerCommand/ActiveTrajectory", trajectory.name());
  }

  @Override
  public void execute() {
    final double timestamp = Timer.getFPGATimestamp();
    final double dt = timestamp - lastTimestamp;
    lastTimestamp = timestamp;

    final boolean flip = AllianceFlipUtil.shouldFlip();
    final Pose2d robotPose = robotState.getPose();

    // Sample at current followingTime to assess error for state machine
    SwerveSample sample =
        trajectory
            .sampleAt(followingTime, flip)
            .orElseGet(() -> trajectory.getFinalSample(flip).orElse(null));

    if (sample == null) {
      drive.stop();
      return;
    }

    Pose2d targetPose = sample.getPose();
    double translationalError = robotPose.getTranslation().getDistance(targetPose.getTranslation());

    // State machine decision based on current error
    updateState(translationalError);

    // Only advance time when following
    if (state == State.Following) {
      followingTime += dt;
      followingTime = MathUtil.clamp(followingTime, 0, trajectory.getTotalTime());

      sample =
          trajectory
              .sampleAt(followingTime, flip)
              .orElseGet(() -> trajectory.getFinalSample(flip).orElse(null));

      if (sample == null) {
        drive.stop();
        return;
      }

      targetPose = sample.getPose();
      translationalError = robotPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    triggerEvents();

    final ChassisSpeeds speeds =
        switch (state) {
          case Following ->
              new ChassisSpeeds(
                  sample.vx + xController.calculate(robotPose.getX(), targetPose.getX()),
                  sample.vy + yController.calculate(robotPose.getY(), targetPose.getY()),
                  sample.omega
                      + headingController.calculate(
                          robotPose.getRotation().getRadians(),
                          targetPose.getRotation().getRadians()));
          case Recovering ->
              new ChassisSpeeds(
                  xController.calculate(robotPose.getX(), targetPose.getX()),
                  yController.calculate(robotPose.getY(), targetPose.getY()),
                  headingController.calculate(
                      robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians()));
        };

    drive.runFieldRelative(speeds);

    Logger.recordOutput("Auto/FollowerCommand/CommandedSpeeds", speeds);

    Logger.recordOutput("Auto/FollowerCommand/TargetPose", targetPose);
    Logger.recordOutput("Auto/FollowerCommand/TrackingState", state);
    Logger.recordOutput("Auto/FollowerCommand/FollowingTimestamp", followingTime);
    Logger.recordOutput("Auto/FollowerCommand/PositionalError", translationalError);
    Logger.recordOutput("Auto/FollowerCommand/EventIndex", nextEventIndex);
  }

  @Override
  public void end(boolean interrupted) {
    finished = !interrupted;
    drive.stop();
    Logger.recordOutput("Odometry/Trajectory", new Pose2d[0]);
    Logger.recordOutput("Odometry/TrajectorySetpoint", new Pose2d());

    Logger.recordOutput("Auto/FollowerCommand/State", "DONE");
  }

  @Override
  public boolean isFinished() {
    if (followingTime < trajectory.getTotalTime()) {
      return false;
    }

    final Pose2d robotPose = robotState.getPose();
    final Pose2d target =
        trajectory.getFinalPose(AllianceFlipUtil.shouldFlip()).orElse(Pose2d.kZero);

    final double error = robotPose.getTranslation().getDistance(target.getTranslation());

    return error < RESUME_THRESHOLD.get();
  }
}
