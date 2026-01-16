package com.frc6324.robot2026.subsystems.vision;

import com.frc6324.lib.UninstantiableClass;
import com.frc6324.lib.util.Statics;
import com.frc6324.robot2026.subsystems.vision.VisionIO.VisionEstimation;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Filesystem;

@UninstantiableClass
public final class VisionConstants {
  private VisionConstants() {
    throw new IllegalAccessError();
  }

  public static final AprilTagFieldLayout APRILTAG_LAYOUT =
      Statics.initOrDefault(
          () ->
              new AprilTagFieldLayout(
                  Filesystem.getDeployDirectory().getAbsolutePath() + "/apriltags.json"),
          // TODO: Make this load 2026 field when it is released
          () -> AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));

  public static final String[] CAMERA_NAMES = {};
  public static final VisionEstimation[] NO_ESTIMATIONS = new VisionEstimation[0];

  public static final boolean HEADING_FREE = false;
  public static final double HEADING_FACTOR = 10;

  public static final double MAX_LATENCY = 2;
}
