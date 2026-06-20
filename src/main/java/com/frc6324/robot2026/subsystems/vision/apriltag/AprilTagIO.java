package com.frc6324.robot2026.subsystems.vision.apriltag;

import com.frc6324.lib.io.IOLayer;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface AprilTagIO extends IOLayer<AprilTagIO.AprilTagInputs> {
  @AutoLog
  class AprilTagInputs {
    public boolean connected = false;
    public VisionEstimation[] estimations = new VisionEstimation[0];
    public int[] tagsSeen = new int[0];
  }

  public record VisionEstimation(
      Pose3d robotPose,
      double timestamp,
      double ambiguity,
      double averageTagDistance,
      int numTagsUsed,
      EstimationStrategy strategy) {}

  public enum EstimationStrategy {
    ConstrainedSolvePNP,
    Multitag,
    Singletag,
  }
}
