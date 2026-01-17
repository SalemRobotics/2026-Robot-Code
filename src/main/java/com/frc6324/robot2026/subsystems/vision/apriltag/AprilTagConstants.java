package com.frc6324.robot2026.subsystems.vision.apriltag;

import com.frc6324.lib.UninstantiableClass;
import com.frc6324.lib.util.Statics;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

@UninstantiableClass
public final class AprilTagConstants {
  private AprilTagConstants() {
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
  public static final double[] CAMERA_STDDEV_FACTORS = {1, 1};

  public static final boolean HEADING_FREE = false;
  public static final double HEADING_FACTOR = 10;

  public static final double LINEAR_STDDEV_BASELINE = Units.inchesToMeters(2);
  public static final double ANGULAR_STDDEV_BASELINE = Units.degreesToRadians(2.5);

  public static final double MAX_LATENCY_SECS = 2;
  public static final double MAX_AMBIGUITY = 0.2;
}
