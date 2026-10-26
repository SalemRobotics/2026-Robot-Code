package com.frc6324.robot2026.subsystems.vision.apriltag;

import static com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagConstants.*;

import com.frc6324.robot2026.RobotState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagIOPhoton implements AprilTagIO {
  @Getter private final CameraProps props;
  private final RobotState robotState = RobotState.getInstance();

  private final Lock cameraLock = new ReentrantLock();
  private final Lock observationLock = new ReentrantLock();

  private final HashSet<Integer> tagsSeen = new HashSet<>();
  private final ArrayList<VisionEstimation> estimations = new ArrayList<>();

  protected final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  private Matrix<N3, N3> cameraMatrix;
  private Matrix<N8, N1> distortionCoefficients;
  private final BooleanSupplier enableSignal;

  /**
   * Cached state of {@link #enableSignal}, updated on the main thread so that the asynchronous
   * vision thread does not have to call potentially non-thread-safe supplier code.
   */
  private volatile boolean enabled = true;

  public AprilTagIOPhoton(CameraProps props, BooleanSupplier enableSignal) {
    this.props = props;
    this.enableSignal = enableSignal;

    this.camera = new PhotonCamera(props.name());
    this.poseEstimator = new PhotonPoseEstimator(APRILTAG_LAYOUT, props.robotToCameraLens());

    this.cameraMatrix = camera.getCameraMatrix().orElse(null);
    this.distortionCoefficients = camera.getDistCoeffs().orElse(null);

    VisionUpdateThread.addCallback(this::updateOdometry);
  }

  /** Helper constructor for sim to force the system to be initialized before the first camera */
  protected AprilTagIOPhoton(CameraProps props, VisionSystemSim sim) {
    this(props);
  }

  public AprilTagIOPhoton(CameraProps props) {
    this(props, () -> true);
  }

  /**
   * Updates the vision odometry for this camera.
   *
   * @implNote This method should be called asynchronously.
   */
  private void updateOdometry() {
    // Avoid doing (and accumulating) work while this camera is disabled. Any observations that
    // were queued before the camera was disabled are dropped so that they cannot be delivered as
    // stale measurements once it is re-enabled.
    if (!enabled) {
      clearObservations();
      return;
    }

    final List<PhotonPipelineResult> allResults;
    final Matrix<N3, N3> cameraMatrix;
    final Matrix<N8, N1> distCoeffs;

    final ArrayList<VisionEstimation> estimations = new ArrayList<>();
    final HashSet<Integer> tagsSeen = new HashSet<>();

    cameraLock.lock();
    try {
      allResults = camera.getAllUnreadResults();

      // Save the cam matrix & dist coeffs here so that we don't have to lock later on
      cameraMatrix = this.cameraMatrix;
      distCoeffs = distortionCoefficients;
    } finally {
      cameraLock.unlock();
    }

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
        final Optional<Pose2d> odomPoseOpt = robotState.samplePoseAt(timestamp);
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
          strategy = EstimationStrategy.ConstrainedSolvePNP;
        } else {
          // It is safe to call `.get` here since we already know that the result has targets
          estimatedPose = poseEstimator.estimateAverageBestTargetsPose(result).get();
          strategy = EstimationStrategy.Singletag;
        }
      }

      final List<PhotonTrackedTarget> targetsUsed = estimatedPose.targetsUsed;
      final int numTags = targetsUsed.size();

      // Defensive guard: an estimation with no targets would make the averages below NaN.
      if (numTags == 0) {
        continue;
      }

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

    observationLock.lock();
    try {
      this.tagsSeen.addAll(tagsSeen);
      this.estimations.addAll(estimations);
    } finally {
      observationLock.unlock();
    }
  }

  @Override
  public void updateInputs(AprilTagInputs inputs) {
    enabled = enableSignal.getAsBoolean();

    if (!camera.isConnected() || !enabled) {
      inputs.connected = false;
      inputs.estimations = new VisionEstimation[0];
      inputs.tagsSeen = new int[0];

      // Discard observations gathered before the camera disconnected or was disabled instead of
      // holding them until it comes back and delivering them as stale measurements.
      clearObservations();

      return;
    }

    cameraLock.lock();
    try {
      if (cameraMatrix == null) {
        cameraMatrix = camera.getCameraMatrix().orElse(null);
      }

      if (distortionCoefficients == null) {
        distortionCoefficients = camera.getDistCoeffs().orElse(null);
      }

    } finally {
      cameraLock.unlock();
    }

    inputs.connected = true;

    observationLock.lock();
    try {
      inputs.estimations = estimations.toArray(VisionEstimation[]::new);
      inputs.tagsSeen = tagsSeen.stream().mapToInt(Integer::intValue).toArray();

      estimations.clear();
      tagsSeen.clear();
    } finally {
      observationLock.unlock();
    }
  }

  /** Clears any observations that have not yet been delivered to the main thread. */
  private void clearObservations() {
    observationLock.lock();
    try {
      estimations.clear();
      tagsSeen.clear();
    } finally {
      observationLock.unlock();
    }
  }

  @FunctionalInterface
  public interface OdometryPoseGetter {
    Optional<Pose2d> samplePoseAt(double timestamp);
  }
}
