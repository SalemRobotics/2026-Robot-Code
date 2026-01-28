package com.frc6324.robot2026.subsystems.vision.objdetect;

import static com.frc6324.robot2026.subsystems.vision.objdetect.ObjectDetectionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public final class ObjDetectIOPhoton implements ObjDetectIO {
  private static int cameraIndex = 0;

  private final int index = cameraIndex++;
  private final PhotonCamera camera = new PhotonCamera(CAMERA_NAMES[index]);
  private final Transform3d robotToCamera = ROBOT_TO_CAMERAS[index];
  private final ArrayList<VisibleGamePiece> pieces = new ArrayList<>();

  @Override
  public void updateInputs(ObjDetectInputs inputs) {
    pieces.clear();

    inputs.connected = camera.isConnected();
    if (!inputs.connected) {
      return;
    }

    for (final PhotonPipelineResult result : camera.getAllUnreadResults()) {
      final double timestamp = result.getTimestampSeconds();

      for (final PhotonTrackedTarget target : result.targets) {
        // Add the camera-to-target vector to the camera's known robot-relative pose
        final Transform3d robotToTarget = robotToCamera.plus(target.bestCameraToTarget);
        final double distance = robotToTarget.getTranslation().getNorm();

        // Compute the robot relative direction to head in to get to the piece
        final Rotation2d theta = robotToTarget.getTranslation().toTranslation2d().getAngle();

        final double ratio = Math.min(1, INTAKE_HALF_WIDTH / distance);
        final double dTheta = Math.asin(ratio);

        pieces.add(
            new VisibleGamePiece(
                timestamp,
                target.poseAmbiguity,
                distance,
                robotToTarget,
                target.objDetectId,
                // Bind the piece's theta to [-π, π] when ±Δθ is applied
                MathUtil.angleModulus(theta.getRadians() - dTheta),
                MathUtil.angleModulus(theta.getRadians() + dTheta)));
      }
    }

    inputs.visiblePieces = pieces.toArray(VisibleGamePiece[]::new);
  }
}
