package com.frc6324.robot2026.subsystems.vision;

import static com.frc6324.robot2026.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleFunction;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhoton implements VisionIO {
  private static int cameraIndex = 0;

  protected final int index = cameraIndex++;
  private final PhotonCamera camera = new PhotonCamera(CAMERA_NAMES[index]);
  private final DoubleFunction<Optional<Pose2d>> robotPoseEstimationBuffer;
  private final Set<Integer> tagsSeen = new HashSet<>();

  private Matrix<N3, N3> cameraMatrix = camera.getCameraMatrix().orElse(null);
  private Matrix<N8, N1> distCoeffs = camera.getDistCoeffs().orElse(null);

  private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(APRILTAG_LAYOUT, null);
  private final ArrayList<VisionEstimation> estimations = new ArrayList<>();

  public VisionIOPhoton(DoubleFunction<Optional<Pose2d>> robotPoseBufferGetter) {
    robotPoseEstimationBuffer = robotPoseBufferGetter;
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    boolean connected = camera.isConnected();
    inputs.connected = connected;

    estimations.clear();
    tagsSeen.clear();

    // If the camera isn't connected, skip
    if (!connected) {
      inputs.estimations = NO_ESTIMATIONS;
      inputs.tagsSeen = new int[0];
      return;
    }

    // Try to initialize the camera matrix if it isn't already
    if (cameraMatrix == null) {
      cameraMatrix = camera.getCameraMatrix().orElse(null);
    }

    // Try to initialize the distance coefficients if it isn't already
    if (distCoeffs == null) {
      distCoeffs = camera.getDistCoeffs().orElse(null);
    }

    // Go over all of the
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      EstimatedRobotPose estimatedPose;
      double timestamp = result.getTimestampSeconds();

      // If the result doesn't have targets or is stale, skip
      if (!result.hasTargets() || Timer.getFPGATimestamp() - timestamp > MAX_LATENCY) {
        continue;
      }

      // Estimation attempt order:
      // 1. Multitag on coprocessor
      // 2. Constrained solvePNP using robot pose estimation
      // 3. Average targets (using ambiguity as weight)

      // Try to estimate using multitag
      Optional<EstimatedRobotPose> multitagOpt = poseEstimator.estimateCoprocMultiTagPose(result);
      if (multitagOpt.isPresent()) {
        estimatedPose = multitagOpt.get();
      } else {
        Optional<Pose2d> odomPoseOpt = robotPoseEstimationBuffer.apply(timestamp);
        Optional<EstimatedRobotPose> constrainedSolvePNPOpt = Optional.empty();

        if (cameraMatrix != null && distCoeffs != null && odomPoseOpt.isPresent()) {
          Pose2d odomPose = odomPoseOpt.get();

          constrainedSolvePNPOpt =
              poseEstimator.estimateConstrainedSolvepnpPose(
                  result,
                  cameraMatrix,
                  distCoeffs,
                  new Pose3d(odomPose),
                  HEADING_FREE,
                  HEADING_FACTOR);
        }

        if (constrainedSolvePNPOpt.isPresent()) {
          estimatedPose = constrainedSolvePNPOpt.get();
        } else {
          // It is safe to call `.get()` here since we already asserted there are targets
          estimatedPose = poseEstimator.estimateAverageBestTargetsPose(result).get();
        }
      }

      List<PhotonTrackedTarget> targetsUsed = estimatedPose.targetsUsed;
      int numTags = targetsUsed.size();

      double totalAmbiguity = 0;
      double totalDistance = 0;

      for (int i = 0; i < numTags; i++) {
        PhotonTrackedTarget target = targetsUsed.get(i);

        tagsSeen.add(target.fiducialId);

        totalAmbiguity += target.poseAmbiguity;
        totalDistance += target.bestCameraToTarget.getTranslation().getNorm();
      }

      estimations.add(
          new VisionEstimation(
              estimatedPose.estimatedPose,
              timestamp,
              totalAmbiguity / numTags,
              totalDistance / numTags,
              numTags));
    }

    inputs.estimations = estimations.toArray(VisionEstimation[]::new);
    inputs.tagsSeen = tagsSeen.stream().mapToInt(i -> i).toArray();
  }
}
