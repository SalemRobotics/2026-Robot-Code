package com.frc6324.robot2026.subsystems.vision.objdetect;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface ObjDetectIO {
  /**
   * Updates this camera's
   *
   * @param inputs
   */
  public void updateInputs(ObjDetectInputs inputs);

  @AutoLog
  class ObjDetectInputs {
    public boolean connected = false;

    /** The list of visible game pieces */
    public VisibleGamePiece[] visiblePieces = new VisibleGamePiece[0];
  }

  /** Represents a game piece that is visible to the camera. */
  record VisibleGamePiece(
      double timestamp,
      double ambiguity,
      double distance,
      Transform3d robotToTarget,
      int objdetectId,
      double startTheta,
      double endTheta) {}
}
