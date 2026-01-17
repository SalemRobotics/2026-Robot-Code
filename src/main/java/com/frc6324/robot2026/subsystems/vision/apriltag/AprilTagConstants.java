package com.frc6324.robot2026.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.Hertz;

import com.frc6324.lib.UninstantiableClass;
import com.frc6324.lib.util.Statics;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Frequency;
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

  public static final String[] CAMERA_NAMES = {
    "Front Left", "Back Left", "Front Right", "Back Right"
  };
  public static final double[] CAMERA_STDDEV_FACTORS = {1, 1, 1, 1};
  public static final Resolution[] CAMERA_RESOLUTIONS = {
    new Resolution(1280, 800),
    new Resolution(1280, 800),
    new Resolution(1280, 800),
    new Resolution(1280, 800)
  };
  public static final double[] CAMERA_LATENCIES = {30, 30, 30, 30};
  public static final double[] CAMERA_FPS = {60, 60, 60, 60};
  public static final Rotation2d[] CAMERA_FOVS = {
    Rotation2d.fromDegrees(70),
    Rotation2d.fromDegrees(110),
    Rotation2d.fromDegrees(70),
    Rotation2d.fromDegrees(110)
  };
  public static final Transform3d[] ROBOT_TO_CAMERAS = {
    new Transform3d(), new Transform3d(), new Transform3d(), new Transform3d()
  };

  public static final Frequency UPDATE_THREAD_FREQUENCY = Hertz.of(30);

  public static final boolean HEADING_FREE = false;
  public static final double HEADING_FACTOR = 10;

  public static final double LINEAR_STDDEV_BASELINE = Units.inchesToMeters(2);
  public static final double ANGULAR_STDDEV_BASELINE = Units.degreesToRadians(2.5);

  public static final double MAX_LATENCY_SECS = 2;
  public static final double MAX_AMBIGUITY = 0.2;

  public record Resolution(int width, int height) {}
}
