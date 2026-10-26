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
  private static final String aprilTagBaseKey = "Vision/AprilTag/";

  private final AprilTagIO[] io;
  private final AprilTagInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private final VisionRule[] rules;
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
    inputs = new AprilTagInputsAutoLogged[io.length];

    for (int i = 0; i < io.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "AprilTag camera '" + CAMERAS[i].name() + "' is disconnected.", AlertType.kWarning);
      inputs[i] = new AprilTagInputsAutoLogged();
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

      final CameraProps props = CAMERAS[cameraIndex];

      final String logKey = aprilTagBaseKey + props.name();
      final AprilTagInputsAutoLogged currentInputs = inputs[cameraIndex];

      io[cameraIndex].updateInputs(currentInputs);
      Logger.processInputs(logKey, currentInputs);
      disconnectedAlerts[cameraIndex].set(!currentInputs.connected);
      Logger.recordOutput(logKey + "/Visible Tag IDs", currentInputs.tagsSeen);

      for (final VisionEstimation estimation : currentInputs.estimations) {
        boolean rejected = false;
        for (final VisionRule rule : rules) {
          if (!rule.shouldPermit(estimation)) {
            rule.reject(estimation); // Logs this pose as rejected by the specific rule
            rejected = true;
          }
        }

        final Pose2d robotPose = estimation.robotPose().toPose2d();
        robotPoses.add(robotPose);

        // Estimations with no tags should never be produced, but guard against them so that the
        // standard-deviation calculation below can never divide by zero.
        if (rejected || estimation.numTagsUsed() <= 0) {
          robotPosesRejected.add(robotPose);
          continue;
        }

        robotPosesAccepted.add(robotPose);

        final double stddevFactor =
            Math.pow(estimation.averageTagDistance(), 2) / estimation.numTagsUsed();

        final double linearStddev =
            LINEAR_STDDEV_BASELINE * stddevFactor * props.standardDeviationFactor();
        final double angularStddev =
            ANGULAR_STDDEV_BASELINE * stddevFactor * props.standardDeviationFactor();

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

    for (final VisionRule rule : rules) {
      rule.logRejected();
    }

    Logger.recordOutput(
        aprilTagBaseKey + "Summary/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));
    Logger.recordOutput(
        aprilTagBaseKey + "Summary/PosesAccepted", allRobotPosesAccepted.toArray(Pose2d[]::new));
    Logger.recordOutput(
        aprilTagBaseKey + "Summary/PosesRejected", allRobotPosesRejected.toArray(Pose2d[]::new));

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
