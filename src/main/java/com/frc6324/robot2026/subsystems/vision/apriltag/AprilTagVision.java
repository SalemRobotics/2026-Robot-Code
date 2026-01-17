package com.frc6324.robot2026.subsystems.vision.apriltag;

import static com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagConstants.*;

import com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagIO.VisionEstimation;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
  private final AprilTagIO[] io;
  private final VisionInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final ArrayList<VisionConsumer> consumers = new ArrayList<>();

  private final ArrayList<Pose2d> allRobotPoses = new ArrayList<>();
  private final ArrayList<Pose2d> allRobotPosesAccepted = new ArrayList<>();
  private final ArrayList<Pose2d> allRobotPosesRejected = new ArrayList<>();

  private final ArrayList<Pose2d> robotPoses = new ArrayList<>();
  private final ArrayList<Pose2d> robotPosesAccepted = new ArrayList<>();
  private final ArrayList<Pose2d> robotPosesRejected = new ArrayList<>();

  public AprilTagVision(AprilTagIO... io) {
    this.io = io;

    disconnectedAlerts = new Alert[io.length];
    inputs = new VisionInputsAutoLogged[io.length];

    for (int i = 0; i < io.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "AprilTag camera '" + CAMERA_NAMES[i] + "' is disconnected.", AlertType.kWarning);
      inputs[i] = new VisionInputsAutoLogged();
    }

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
        final int numTagsUsed = estimation.numTagsUsed();
        final Pose2d robotPose = estimation.robotPose().toPose2d();

        // Determine whether to reject pose if:
        // 1. The update (somehow) uses 0 tags (should be impossible)
        // 2. The update has too high of an ambiguity and is single-tag
        // 3. The pose is outside of the field border
        final boolean rejectPose =
            numTagsUsed == 0
                || (numTagsUsed == 1 && estimation.ambiguity() > MAX_AMBIGUITY)
                || robotPose.getX() < 0
                || robotPose.getX() > APRILTAG_LAYOUT.getFieldLength()
                || robotPose.getY() < 0
                || robotPose.getY() > APRILTAG_LAYOUT.getFieldWidth();

        // Add the robot pose to the "all poses" cache
        robotPoses.add(robotPose);

        // Add the pose to its respective cache
        if (rejectPose) {
          robotPosesRejected.add(robotPose);

          // Skip stddev calculations
          continue;
        } else {
          robotPosesAccepted.add(robotPose);
        }

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

    // Log vision summary data
    Logger.recordOutput("Vision/AprilTag/Summary/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));
    Logger.recordOutput(
        "Vision/AprilTag/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose2d[]::new));
    Logger.recordOutput(
        "Vision/AprilTag/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(Pose2d[]::new));
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
