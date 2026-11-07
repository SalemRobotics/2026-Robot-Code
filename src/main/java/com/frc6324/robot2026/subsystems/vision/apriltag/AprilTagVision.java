package com.frc6324.robot2026.subsystems.vision.apriltag;

import static com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagConstants.*;

import com.frc6324.lib.util.LoggedTracer;
import com.frc6324.lib.util.VirtualSubsystem;
import com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagIO.VisionEstimation;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends VirtualSubsystem {
  private static final String visionBaseKey = "Vision/AprilTag/";

  private final AprilTagIO[] io;
  private final VisionInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final ArrayList<VisionConsumer> consumers = new ArrayList<>();
  private final VisionRule[] rules;

  // Reused every loop. Poses are logged as Pose2d (WPILib struct, cheap to serialize) instead of
  // VisionEstimation records, which AdvantageKit packs with reflection and was logging ~6x each.
  private final ArrayList<Pose2d> posesAccepted = new ArrayList<>();
  private final ArrayList<Pose2d> posesRejected = new ArrayList<>();

  public AprilTagVision(VisionRule[] visionRules, AprilTagIO... io) {
    this.rules = visionRules;
    this.io = io;

    disconnectedAlerts = new Alert[io.length];
    inputs = new VisionInputsAutoLogged[io.length];

    for (int i = 0; i < io.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "AprilTag camera '" + CAMERA_NAMES[i] + "' is disconnected.", AlertType.kWarning);
      inputs[i] = new VisionInputsAutoLogged();
    }

    final Pose3d[] allTagPoses =
        APRILTAG_LAYOUT.getTags().stream().map(tag -> tag.pose).toArray(Pose3d[]::new);

    Logger.recordOutput("Vision/AllKnownAprilTags", allTagPoses);

    VisionUpdateThread.start();
  }

  @Override
  public void periodic() {
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      posesAccepted.clear();
      posesRejected.clear();

      final String logKey = visionBaseKey + CAMERA_NAMES[cameraIndex];
      final VisionInputsAutoLogged currentInputs = inputs[cameraIndex];

      io[cameraIndex].updateInputs(currentInputs);
      Logger.processInputs(logKey, currentInputs);
      Logger.recordOutput(logKey + "/Visible Tag IDs", currentInputs.tagsSeen);

      for (final VisionEstimation estimation : currentInputs.estimations) {
        boolean rejected = false;
        for (final VisionRule rule : rules) {
          if (!rule.shouldPermit(estimation)) {
            rule.reject(estimation); // Counts this pose as rejected by the specific rule
            rejected = true;
            // Skip the rest of the rules
            break;
          }
        }

        final Pose2d robotPose = estimation.robotPose().toPose2d();

        if (rejected) {
          posesRejected.add(robotPose);
          continue;
        }

        posesAccepted.add(robotPose);

        final double stddevFactor =
            Math.pow(estimation.averageTagDistance(), 2) / estimation.numTagsUsed();

        final double linearStddev =
            LINEAR_STDDEV_BASELINE * stddevFactor * CAMERA_STDDEV_FACTORS[cameraIndex];
        final double angularStddev =
            ANGULAR_STDDEV_BASELINE * stddevFactor * CAMERA_STDDEV_FACTORS[cameraIndex];

        // Send the update to all consumers
        consumers.forEach(
            c ->
                c.addVisionMeasurement(
                    robotPose,
                    estimation.timestamp(),
                    VecBuilder.fill(linearStddev, linearStddev, angularStddev)));
      }

      // Log camera summary data
      Logger.recordOutput(logKey + "/PosesAccepted", posesAccepted.toArray(Pose2d[]::new));
      Logger.recordOutput(logKey + "/PosesRejected", posesRejected.toArray(Pose2d[]::new));
    }

    for (final VisionRule rule : rules) {
      rule.logRejected();
    }

    LoggedTracer.record("Periodic/AprilTag periodic");
  }

  /**
   * Adds a consumer that will accept updates deemed acceptable from vision odometry.
   *
   * @param consumer The consumer to add.
   * @return This subsystem for easier method chaining.
   */
  public AprilTagVision withConsumer(VisionConsumer consumer) {
    consumers.add(consumer);
    return this;
  }

  /** Marks a consumer of vision data. */
  @FunctionalInterface
  public interface VisionConsumer {
    /**
     * Adds a vision measurement to this consumer.
     *
     * @param robotPose The calculated robot pose.
     * @param timestamp The timestamp of the pose measurement.
     * @param stddevs The calculated standard deviations of robot pose calculations.
     */
    public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stddevs);
  }
}
