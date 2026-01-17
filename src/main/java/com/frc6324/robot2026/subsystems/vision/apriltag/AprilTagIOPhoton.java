package com.frc6324.robot2026.subsystems.vision.apriltag;

import static com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagConstants.*;

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
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagIOPhoton implements AprilTagIO {
  private static int cameraIndex = 0;

  protected final int index = cameraIndex++;
  private final OdometryPoseGetter odometryPoseAtTime;

  private final Lock updateLock = new ReentrantLock();
  private final AtomicReference<HashSet<Integer>> tagsSeen = new AtomicReference<>(new HashSet<>());
  private final AtomicReference<ArrayList<VisionEstimation>> estimations =
      new AtomicReference<>(new ArrayList<>());

  protected final PhotonCamera camera = new PhotonCamera(CAMERA_NAMES[index]);
  private final PhotonPoseEstimator poseEstimator =
      new PhotonPoseEstimator(APRILTAG_LAYOUT, ROBOT_TO_CAMERAS[index]);

  private Matrix<N3, N3> cameraMatrix = camera.getCameraMatrix().orElse(null);
  private Matrix<N8, N1> distortionCoefficients = camera.getDistCoeffs().orElse(null);

  public AprilTagIOPhoton(OdometryPoseGetter odometryPoseGetter) {
    odometryPoseAtTime = odometryPoseGetter;

    VisionUpdateThread.addCallback(this::updateOdometry);
  }

  /**
   * Updates the vision odometry for this camera.
   *
   * @implNote This method should be called asynchronously.
   */
  private void updateOdometry() {
    final List<PhotonPipelineResult> allResults;
    final Matrix<N3, N3> cameraMatrix;
    final Matrix<N8, N1> distCoeffs;

    final ArrayList<VisionEstimation> estimations = new ArrayList<>();
    final HashSet<Integer> tagsSeen = new HashSet<>();

    updateLock.lock();
    try {
      allResults = camera.getAllUnreadResults();

      // Save the cam matrix & dist coeffs here so that we don't have to lock later on
      cameraMatrix = this.cameraMatrix;
      distCoeffs = distortionCoefficients;
    } finally {
      updateLock.unlock();
    }

    for (final PhotonPipelineResult result : allResults) {
      final double timestamp = result.getTimestampSeconds();
      final EstimatedRobotPose estimatedPose;

      // If the result doesn't have targets or is stale, skip it
      if (!result.hasTargets() || Timer.getFPGATimestamp() - timestamp > MAX_LATENCY_SECS) {
        continue;
      }

      // Estimation attempt order:
      // 1. Multitag on the coprocessor
      // 2. Constrained SolvePNP using robot pose estimation
      // 3. Average of targets using ambiguity as weight
      final Optional<EstimatedRobotPose> multitagOpt =
          poseEstimator.estimateCoprocMultiTagPose(result);
      if (multitagOpt.isPresent()) {
        estimatedPose = multitagOpt.get();
      } else {
        final Optional<Pose2d> odomPoseOpt = odometryPoseAtTime.samplePoseAt(timestamp);
        Optional<EstimatedRobotPose> constrainedSolvePNPOpt = Optional.empty();

        if (cameraMatrix != null && distCoeffs != null && odomPoseOpt.isPresent()) {
          final Pose2d odomPose = odomPoseOpt.get();

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
          // It is safe to call `.get` here since we already know that the result has targets
          estimatedPose = poseEstimator.estimateAverageBestTargetsPose(result).get();
        }
      }

      final List<PhotonTrackedTarget> targetsUsed = estimatedPose.targetsUsed;
      final int numTags = targetsUsed.size();

      double totalAmbiguity = 0;
      double totalDistance = 0;

      for (int i = 0; i < numTags; i++) {
        final PhotonTrackedTarget target = targetsUsed.get(i);

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

    this.tagsSeen.set(tagsSeen);
    this.estimations.set(estimations);
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    if (!camera.isConnected()) {
      inputs.connected = false;
      inputs.estimations = new VisionEstimation[0];
      inputs.tagsSeen = new int[0];

      return;
    }

    updateLock.lock();
    try {
      if (cameraMatrix == null) {
        cameraMatrix = camera.getCameraMatrix().orElse(null);
      }

      if (distortionCoefficients == null) {
        distortionCoefficients = camera.getDistCoeffs().orElse(null);
      }

      inputs.connected = true;
      inputs.estimations = estimations.get().toArray(VisionEstimation[]::new);
      inputs.tagsSeen = tagsSeen.get().stream().mapToInt(i -> i).toArray();
    } finally {
      updateLock.unlock();
    }
  }

  @FunctionalInterface
  public interface OdometryPoseGetter {
    Optional<Pose2d> samplePoseAt(double timestamp);
  }
}
