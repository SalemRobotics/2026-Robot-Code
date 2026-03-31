package com.frc6324.robot2026.subsystems.vision.apriltag;

import static com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagConstants.*;

import com.frc6324.lib.util.PoseExtensions.PoseSupplier;
import com.frc6324.robot2026.subsystems.drive.DrivingUtils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
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
  private final PoseSupplier robotPoseGetter;

  private final Lock updateLock = new ReentrantLock();
  private final AtomicReference<CameraState> state =
      new AtomicReference<>(new CameraState(Rotation2d.kZero, 0));

  protected final PhotonCamera camera = new PhotonCamera(CAMERA_NAMES[index]);
  private final PhotonPoseEstimator poseEstimator =
      new PhotonPoseEstimator(APRILTAG_LAYOUT, ROBOT_TO_CAMERAS[index]);

  private Matrix<N3, N3> cameraMatrix = camera.getCameraMatrix().orElse(null);
  private Matrix<N8, N1> distortionCoefficients = camera.getDistCoeffs().orElse(null);
  private final BooleanSupplier enableSignal;

  public AprilTagIOPhoton(PoseSupplier robotPoseSupplier, BooleanSupplier enableSignal) {
    this.robotPoseGetter = robotPoseSupplier;
    this.enableSignal = enableSignal;

    VisionUpdateThread.addCallback(this::updateOdometry);
  }

  public AprilTagIOPhoton(PoseSupplier robotPoseSupplier) {
    this(robotPoseSupplier, () -> true);
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

    updateLock.lock();
    try {
      allResults = camera.getAllUnreadResults();

      // Save the cam matrix & dist coeffs here so that we don't have to lock later on
      cameraMatrix = this.cameraMatrix;
      distCoeffs = distortionCoefficients;
    } finally {
      updateLock.unlock();
    }

    final CameraState state = this.state.get();
    if (!state.addedHeadingData) {
      poseEstimator.addHeadingData(state.headingTimestamp, state.lastHeading);
      state.addedHeadingData = true;
    }

    for (final PhotonPipelineResult result : allResults) {
      final double timestamp = result.getTimestampSeconds();

      // If the result doesn't have targets or is stale, skip it
      if (!result.hasTargets() || Timer.getFPGATimestamp() - timestamp > MAX_LATENCY_SECS) {
        continue;
      }

      final EstimatedRobotPose estimatedPose;
      final EstimationStrategy strategy;

      final Optional<EstimatedRobotPose> multitagOpt =
          poseEstimator.estimateCoprocMultiTagPose(result);
      if (multitagOpt.isPresent()) {
        estimatedPose = multitagOpt.get();
        strategy = EstimationStrategy.Multitag;
      } else if (DrivingUtils.isTilted()) {
        final Optional<EstimatedRobotPose> lowestAmbiguity =
            poseEstimator.estimateLowestAmbiguityPose(result);
        if (lowestAmbiguity.isPresent()) {
          estimatedPose = lowestAmbiguity.get();
          strategy = EstimationStrategy.LowestAmbiguity;
        } else continue;
      } else {
        final Optional<EstimatedRobotPose> seedOpt =
            poseEstimator.estimatePnpDistanceTrigSolvePose(result);
        if (seedOpt.isEmpty()) {
          continue;
        }

        final EstimatedRobotPose seed = seedOpt.get();

        // Try to use the seed to
        Optional<EstimatedRobotPose> constrainedSolvePNPOpt = Optional.empty();
        if (cameraMatrix != null && distCoeffs != null) {
          final boolean disabled = DriverStation.isDisabled();

          constrainedSolvePNPOpt =
              poseEstimator.estimateConstrainedSolvepnpPose(
                  result,
                  cameraMatrix,
                  distCoeffs,
                  seed.estimatedPose,
                  disabled ? true : HEADING_FREE,
                  disabled ? 0 : HEADING_FACTOR);
        }

        if (constrainedSolvePNPOpt.isPresent()) {
          // If the refined pose exists, use it.
          estimatedPose = constrainedSolvePNPOpt.get();
          strategy = EstimationStrategy.ConstrainedSolvePNP;
        } else {
          // Otherwise, default to the seed.
          estimatedPose = seed;
          strategy = EstimationStrategy.PnpDistanceTrigSolve;
        }
      }

      final List<PhotonTrackedTarget> targetsUsed = estimatedPose.targetsUsed;
      final int numTags = targetsUsed.size();

      double totalAmbiguity = 0;
      double totalDistance = 0;

      for (int i = 0; i < numTags; i++) {
        final PhotonTrackedTarget target = targetsUsed.get(i);

        state.tagsSeen.add(target.fiducialId);

        totalAmbiguity += target.poseAmbiguity;
        totalDistance += target.bestCameraToTarget.getTranslation().getNorm();
      }

      state.estimations.add(
          new VisionEstimation(
              estimatedPose.estimatedPose,
              timestamp,
              totalAmbiguity / numTags,
              totalDistance / numTags,
              numTags,
              strategy));
    }
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    if (!camera.isConnected()) {
      inputs.connected = false;
      inputs.enabled = false;
      inputs.estimations = new VisionEstimation[0];
      inputs.tagsSeen = new int[0];
    }

    if (!enableSignal.getAsBoolean()) {
      inputs.connected = true;
      inputs.enabled = false;
      inputs.estimations = new VisionEstimation[0];
      inputs.tagsSeen = new int[0];
    }

    updateLock.lock();
    try {
      if (cameraMatrix == null) {
        cameraMatrix = camera.getCameraMatrix().orElse(null);
      }

      if (distortionCoefficients == null) {
        distortionCoefficients = camera.getDistCoeffs().orElse(null);
      }

      final CameraState lastState = state.get();
      inputs.connected = true;
      inputs.estimations = lastState.estimations.toArray(VisionEstimation[]::new);
      inputs.tagsSeen = lastState.tagsSeen.stream().mapToInt(i -> i).toArray();

      final double timestamp = Timer.getTimestamp();
      final Rotation2d currentRot = robotPoseGetter.getPose().getRotation();
      state.set(new CameraState(currentRot, timestamp));
    } finally {
      updateLock.unlock();
    }
  }

  private class CameraState {
    final ArrayList<VisionEstimation> estimations = new ArrayList<>();
    final HashSet<Integer> tagsSeen = new HashSet<>();

    boolean addedHeadingData = false;
    final Rotation2d lastHeading;
    final double headingTimestamp;

    public CameraState(Rotation2d heading, double timestamp) {
      lastHeading = heading;
      headingTimestamp = timestamp;
    }
  }
}
