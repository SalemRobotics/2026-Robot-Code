package com.frc6324.robot2026.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.util.FieldConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Frequency;
import lombok.experimental.UtilityClass;

@UtilityClass
public final class AprilTagConstants {
  public static final AprilTagFieldLayout APRILTAG_LAYOUT =
      FieldConstants.DEFAULT_APRILTAG_TYPE.getLayout();

  public static final CameraProps[] CAMERAS = {
    new CameraProps(
        "Bottom Right USB",
        1,
        new Resolution(1280, 800),
        20,
        120,
        Rotation2d.fromDegrees(93),
        new Transform3d(
            Units.inchesToMeters(-13.875),
            Units.inchesToMeters(5.25),
            Units.inchesToMeters(15.5),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-157.5)))),
    new CameraProps(
        "Bottom Left USB",
        1,
        new Resolution(1280, 800),
        20,
        120,
        Rotation2d.fromDegrees(93),
        new Transform3d(
            Units.inchesToMeters(-13.875),
            Units.inchesToMeters(-5.25),
            Units.inchesToMeters(15.5),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(157.5)))),
    new CameraProps(
        "Top Right USB",
        1,
        new Resolution(1280, 800),
        20,
        120,
        Rotation2d.fromDegrees(80),
        new Transform3d(
            Units.inchesToMeters(3.75),
            Units.inchesToMeters(12.5),
            Units.inchesToMeters(8.75),
            new Rotation3d(Units.degreesToRadians(10), 0, Units.degreesToRadians(90))))
  };

  public static final Frequency UPDATE_THREAD_FREQUENCY = Hertz.of(60);

  public static final boolean HEADING_FREE = false;
  public static final double HEADING_FACTOR = 10;

  public static final double LINEAR_STDDEV_BASELINE = Units.inchesToMeters(0.2);
  public static final double ANGULAR_STDDEV_BASELINE = Units.degreesToRadians(0.75);

  public static final double MAX_LATENCY_SECS = 2;
  public static final double MAX_AMBIGUITY = 0.2;

  public record CameraProps(
      String name,
      double standardDeviationFactor,
      Resolution resolution,
      double averageLatency,
      double FPS,
      Rotation2d horizontalFOV,
      Transform3d robotToCameraLens) {}

  public record Resolution(int width, int height) {}
}
