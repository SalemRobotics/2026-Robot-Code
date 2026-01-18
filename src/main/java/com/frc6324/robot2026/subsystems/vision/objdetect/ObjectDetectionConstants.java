package com.frc6324.robot2026.subsystems.vision.objdetect;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import com.frc6324.lib.UninstantiableClass;
import com.frc6324.robot2026.subsystems.intake.IntakeConstants;

import edu.wpi.first.math.geometry.Transform3d;

@UninstantiableClass
public final class ObjectDetectionConstants {
  private ObjectDetectionConstants() {
    throw new IllegalAccessError();
  }

  public static final String[] CAMERA_NAMES = { "Intake Camera" };
  public static final Transform3d[] ROBOT_TO_CAMERAS = { new Transform3d() };

  public static final double INTAKE_HALF_WIDTH = IntakeConstants.INTAKE_WIDTH.minus(Centimeters.of(15)).in(Meters) / 2;
}
