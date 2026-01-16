package com.frc6324.robot2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  void updateInputs(VisionInputs inputs);

  @AutoLog
  class VisionInputs {
    public boolean connected = false;
    public VisionEstimation[] estimations = VisionConstants.NO_ESTIMATIONS;
    public int[] tagsSeen = new int[0];
  }

  public record VisionEstimation(
      Pose3d robotPose,
      double timestamp,
      double ambiguity,
      double averageTagDistance,
      int numTagsUsed) {}
}
