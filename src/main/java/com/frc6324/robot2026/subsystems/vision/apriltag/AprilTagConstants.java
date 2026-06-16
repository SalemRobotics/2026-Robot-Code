package com.frc6324.robot2026.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.UninstantiableClass;
import com.frc6324.lib.util.FieldConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Frequency;

@UninstantiableClass
public final class AprilTagConstants {
  private AprilTagConstants() {
    throw new IllegalAccessError();
  }

  public static final AprilTagFieldLayout APRILTAG_LAYOUT =
      FieldConstants.DEFAULT_APRILTAG_TYPE.getLayout();

  public static final CameraProps[] CAMERAS = {
    new CameraProps(
      "Right Drum Camera", 
      1, 
      new Resolution(1280, 800), 
      30, 
      40, 
      Rotation2d.fromDegrees(110), 
      new Transform3d(
        Inches.of(-13.5),
        Inches.of(5.0),
        Inches.of(15.5),
        new Rotation3d(
          Degrees.zero(),
          Degrees.of(-15),
          Degrees.of(157.5))
      )),
    new CameraProps(
      "Left Drum Camera", 
      1, 
      new Resolution(1280, 800), 
      30, 
      40, 
      Rotation2d.fromDegrees(110), 
      new Transform3d(
        Inches.of(-13.5),
        Inches.of(-5.0),
        Inches.of(15.5),
        new Rotation3d(
          Degrees.zero(),
          Degrees.of(-15),
          Degrees.of(-157.5))
      )),
    new CameraProps(
      "Left Aux Camera", 
      1.125, 
      new Resolution(1280, 800), 
      30,
      40, 
      Rotation2d.fromDegrees(80), 
      new Transform3d(
        Inches.of(3.75),
        Inches.of(12.5),
        Inches.of(8.75),
        new Rotation3d(
            Degrees.of(10),
            Degrees.zero(),
            Degrees.of(90))
      ))
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
    Transform3d robotToCameraLens) {
  }

  public record Resolution(int width, int height) {}
}
