package com.frc6324.robot2026.subsystems.vision.apriltag;

import static com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagIOPhoton implements AprilTagIO {
  private static int cameraIndex = 0;

  protected final int index = cameraIndex++;

  // Estimations produced by the vision thread and not yet consumed by updateInputs. The vision
  // thread appends; updateInputs takes everything and clears it, so a batch is never dropped or
  // delivered twice no matter how the two loops interleave. Guarded by updateLock.
  private final Lock updateLock = new ReentrantLock();
  private final ArrayList<VisionEstimation> pendingEstimations = new ArrayList<>();
  private final HashSet<Integer> pendingTagsSeen = new HashSet<>();

  protected final PhotonCamera camera = new PhotonCamera(CAMERA_NAMES[index]);
  private final PhotonPoseEstimator poseEstimator =
      new PhotonPoseEstimator(APRILTAG_LAYOUT, ROBOT_TO_CAMERAS[index]);

  private final BooleanSupplier enableSignal;

  public AprilTagIOPhoton(BooleanSupplier enableSignal) {
    this.enableSignal = enableSignal;

    VisionUpdateThread.addCallback(this::updateOdometry);
  }

  public AprilTagIOPhoton() {
    this(() -> true);
  }

  /**
   * Updates the vision odometry for this camera.
   *
   * @implNote This method should be called asynchronously.
   */
  private void updateOdometry() {
    final List<PhotonPipelineResult> allResults;

    final ArrayList<VisionEstimation> estimations = new ArrayList<>();
    final HashSet<Integer> tagsSeen = new HashSet<>();

    // Only this thread reads results from the camera, so this needs no lock
    allResults = camera.getAllUnreadResults();

    for (final PhotonPipelineResult result : allResults) {
      final double timestamp = result.getTimestampSeconds();
      final EstimatedRobotPose estimatedPose;
      final EstimationStrategy strategy;

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
        strategy = EstimationStrategy.Multitag;
      } else {
        // It is safe to call `.get` here since we already know that the result has targets
        estimatedPose = poseEstimator.estimateAverageBestTargetsPose(result).get();
        strategy = EstimationStrategy.Singletag;
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
              numTags,
              strategy));
    }

    if (estimations.isEmpty() && tagsSeen.isEmpty()) {
      return;
    }

    updateLock.lock();
    try {
      pendingEstimations.addAll(estimations);
      pendingTagsSeen.addAll(tagsSeen);
    } finally {
      updateLock.unlock();
    }
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    final boolean active = camera.isConnected() && enableSignal.getAsBoolean();

    updateLock.lock();
    try {
      if (active) {
        inputs.estimations = pendingEstimations.toArray(VisionEstimation[]::new);
        inputs.tagsSeen = pendingTagsSeen.stream().mapToInt(i -> i).toArray();
      }

      // Always clear, so estimations queued while inactive aren't delivered later as stale data
      pendingEstimations.clear();
      pendingTagsSeen.clear();
    } finally {
      updateLock.unlock();
    }

    inputs.connected = active;
    if (!active) {
      inputs.estimations = new VisionEstimation[0];
      inputs.tagsSeen = new int[0];
    }
  }

  @FunctionalInterface
  public interface OdometryPoseGetter {
    Optional<Pose2d> samplePoseAt(double timestamp);
  }
}
