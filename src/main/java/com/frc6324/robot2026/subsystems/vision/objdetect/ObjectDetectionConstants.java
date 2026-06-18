package com.frc6324.robot2026.subsystems.vision.objdetect;

import static edu.wpi.first.units.Units.*;

import com.frc6324.robot2026.subsystems.intake.IntakeConstants;
import edu.wpi.first.math.geometry.Transform3d;
import lombok.experimental.UtilityClass;

@UtilityClass
public final class ObjectDetectionConstants {
  public static final String[] CAMERA_NAMES = {"Intake Camera"};
  public static final Transform3d[] ROBOT_TO_CAMERAS = {new Transform3d()};

  public static final double INTAKE_HALF_WIDTH =
      IntakeConstants.INTAKE_WIDTH.minus(Centimeters.of(15)).in(Meters) / 2;
}
