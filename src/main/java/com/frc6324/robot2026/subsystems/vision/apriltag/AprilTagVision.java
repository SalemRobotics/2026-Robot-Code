package com.frc6324.robot2026.subsystems.vision.apriltag;

import static com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagConstants.*;

import com.frc6324.lib.util.VirtualSubsystem;
import com.frc6324.lib.util.logging.LoggedTracer;
import com.frc6324.robot2026.RobotState;
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
  private final AprilTagIO[] io;
  private final VisionInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final RobotState robotState = RobotState.getInstance();

  private final ArrayList<Pose2d> allRobotPoses = new ArrayList<>();
  private final ArrayList<Pose2d> allRobotPosesAccepted = new ArrayList<>();
  private final ArrayList<Pose2d> allRobotPosesRejected = new ArrayList<>();

  private final ArrayList<Pose2d> robotPoses = new ArrayList<>();
  private final ArrayList<Pose2d> robotPosesAccepted = new ArrayList<>();
  private final ArrayList<Pose2d> robotPosesRejected = new ArrayList<>();

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
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();

      final String logKey = "Vision/AprilTag/" + CAMERA_NAMES[cameraIndex];
      final VisionInputsAutoLogged currentInputs = inputs[cameraIndex];

      io[cameraIndex].updateInputs(currentInputs);
      Logger.processInputs(logKey, currentInputs);
      Logger.recordOutput(logKey + "/Visible Tag IDs", currentInputs.tagsSeen);

      for (final VisionEstimation estimation : currentInputs.estimations) {
        boolean rejected = false;
        for (final VisionRule rule : rules) {
          if (!rule.shouldPermit(estimation)) {
            rule.reject(estimation); // Logs this pose as rejected by the specific rule
            rejected = true;
            // Skip the rest of the rules
            break;
          }
        }

        final Pose2d robotPose = estimation.robotPose().toPose2d();
        robotPoses.add(robotPose);

        if (rejected) {
          continue;
        } else {
          robotPosesAccepted.add(robotPose);
        }

        // Add the robot pose to the "all poses" cache
        robotPoses.add(robotPose);

        final double stddevFactor =
            Math.pow(estimation.averageTagDistance(), 2) / estimation.numTagsUsed();

        final double linearStddev =
            LINEAR_STDDEV_BASELINE * stddevFactor * CAMERA_STDDEV_FACTORS[cameraIndex];
        final double angularStddev =
            ANGULAR_STDDEV_BASELINE * stddevFactor * CAMERA_STDDEV_FACTORS[cameraIndex];

        // Send the update to all consumers
        robotState.addVisionMeasurement(
            robotPose,
            estimation.timestamp(),
            VecBuilder.fill(linearStddev, linearStddev, angularStddev));
      }

      // Log camera summary data
      Logger.recordOutput(logKey + "/Summary/RobotPoses", robotPoses.toArray(Pose2d[]::new));
      Logger.recordOutput(
          logKey + "/Summary/RobotPosesAccepted", robotPosesAccepted.toArray(Pose2d[]::new));
      Logger.recordOutput(
          logKey + "/Summary/RobotPosesRejected", robotPosesRejected.toArray(Pose2d[]::new));

      // Add summary data to global summary cache
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    LoggedTracer.record("Periodic/AprilTag periodic");
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
