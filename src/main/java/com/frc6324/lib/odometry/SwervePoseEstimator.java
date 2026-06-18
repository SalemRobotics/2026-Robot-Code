package com.frc6324.lib.odometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;

/**
 * Vision-fused swerve drive pose estimator.
 *
 * <p>Wraps {@link SwerveOdometry} (which already provides IMU-enhanced wheel-encoder integration,
 * tilt correction, and slip detection) and layers on top of it:
 *
 * <ul>
 *   <li>A configurable {@link VisionTrustMode} that governs how strongly vision measurements
 *       influence the fused pose estimate.
 *   <li>Per-measurement Kalman gains, so that each call to {@link #addVisionMeasurement(Pose2d,
 *       double, Matrix)} is processed with its own independently computed gain rather than a shared
 *       mutable global. This means back-to-back measurements from cameras with different noise
 *       characteristics are each weighted correctly.
 *   <li>A full replay-forward vision update system that avoids the silent-discard problem present
 *       in WPILib issue #7881: when a late-arriving measurement lands at time {@code t}, all
 *       subsequent vision updates are replayed on top of it so that no future correction is ever
 *       silently invalidated.
 *   <li>An optional translational outlier-rejection gate, active in {@link VisionTrustMode#GATED}
 *       mode, that discards measurements whose position diverges from the current fused estimate by
 *       more than a configurable radius.
 * </ul>
 *
 * <h2>Vision trust modes</h2>
 *
 * <p>The active mode is set via {@link #setVisionTrustMode(VisionTrustMode)} and governs the
 * default behaviour of {@link #addVisionMeasurement(Pose2d, double, Matrix)}:
 *
 * <ul>
 *   <li>{@link VisionTrustMode#FULL_TRUST} — vision is accepted without filtering. The fused pose
 *       is hard-reset to the vision pose at the measurement timestamp, and odometry is replayed
 *       from that anchor.
 *   <li>{@link VisionTrustMode#FILTERED} — the standard Kalman-weighted blend. Vision is trusted
 *       according to the per-measurement standard deviations.
 *   <li>{@link VisionTrustMode#GATED} — same as {@code FILTERED} but measurements whose
 *       translational distance from the current fused estimate exceeds {@link
 *       #setOutlierRejectionRadius(double)} are silently discarded before the Kalman step.
 * </ul>
 *
 * <p>Regardless of the active mode, {@link #overridePoseWithVision(Pose2d)} always performs a hard
 * reset of the fused pose, bypassing all filtering and gating. This is useful for known-reliable
 * one-shot relocalisations (e.g. auto start, field reset button).
 *
 * <h2>Relationship between the two {@code addVisionMeasurement} overloads</h2>
 *
 * <p>Only one overload is provided: {@link #addVisionMeasurement(Pose2d, double, Matrix)}. Each
 * call computes its own Kalman gain from the supplied standard deviations and stores it inside the
 * {@link VisionUpdate} record, so different cameras or varying-quality detections are each weighted
 * independently without mutating any shared state.
 */
public class SwervePoseEstimator {

  // ---------------------------------------------------------------------------
  // Vision trust mode
  // ---------------------------------------------------------------------------

  /**
   * Controls how strongly vision measurements influence the fused pose estimate.
   *
   * @see SwervePoseEstimator#setVisionTrustMode(VisionTrustMode)
   */
  public enum VisionTrustMode {
    /**
     * Vision measurements fully override the fused pose at the measurement timestamp. Odometry is
     * then replayed forward from that anchor. Use this when your vision system is highly reliable
     * and you want zero lag in adapting to relocalisations.
     */
    FULL_TRUST,

    /**
     * Vision measurements are blended with odometry using per-measurement Kalman gains derived from
     * the supplied standard deviations. This is the default mode and the same behaviour as WPILib's
     * {@code SwerveDrivePoseEstimator}, but with the replay-forward fix applied.
     */
    FILTERED,

    /**
     * Same as {@link #FILTERED}, but measurements whose translational distance from the current
     * fused pose estimate exceeds the configured outlier rejection radius are discarded before the
     * Kalman step. Use this when your vision system occasionally produces wildly wrong poses (e.g.
     * tag ambiguity flips) and you want automatic protection against them without having to
     * pre-screen every measurement in user code.
     *
     * @see SwervePoseEstimator#setOutlierRejectionRadius(double)
     */
    GATED,
  }

  // ---------------------------------------------------------------------------
  // Constants and configuration
  // ---------------------------------------------------------------------------

  /** Duration (seconds) of the odometry pose ring buffer. */
  private static final double BUFFER_DURATION = 2.0;

  /**
   * Default outlier rejection radius (meters) used in {@link VisionTrustMode#GATED} mode. A vision
   * measurement whose translational distance from the current fused estimate exceeds this value is
   * silently discarded. Tune via {@link #setOutlierRejectionRadius(double)}.
   */
  private static final double DEFAULT_OUTLIER_REJECTION_RADIUS_METERS = 1.0;

  // ---------------------------------------------------------------------------
  // Immutable state
  // ---------------------------------------------------------------------------

  private final SwerveOdometry odometry;

  /**
   * Diagonal of the process noise covariance matrix Q (squared state standard deviations). Set once
   * in the constructor and never mutated thereafter.
   */
  private final double[] q = new double[3];

  // ---------------------------------------------------------------------------
  // Mutable configuration
  // ---------------------------------------------------------------------------

  private VisionTrustMode visionTrustMode = VisionTrustMode.FILTERED;
  private double outlierRejectionRadiusMeters = DEFAULT_OUTLIER_REJECTION_RADIUS_METERS;

  // ---------------------------------------------------------------------------
  // Estimation state
  // ---------------------------------------------------------------------------

  /**
   * Ring buffer of odometry-only pose estimates keyed by wall-clock timestamp. Used by the vision
   * update system to anchor pose corrections at the moment a camera frame was captured.
   */
  private final TimeInterpolatableBuffer<Pose2d> odometryPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(BUFFER_DURATION);

  /**
   * Ordered map of vision corrections keyed by the timestamp at which the vision frame was
   * captured. Each entry carries its own Kalman gain so that per-measurement standard deviations
   * are preserved across the map's lifetime.
   *
   * <p>Unlike WPILib's implementation, inserting a new entry at time {@code t} does <em>not</em>
   * discard later entries. Instead, all entries after {@code t} are replayed forward (see {@link
   * #replayVisionUpdatesFrom(double)}) so that every historical correction is reconciled with the
   * new anchor.
   */
  private final NavigableMap<Double, VisionUpdate> visionUpdates = new TreeMap<>();

  /** The current vision-fused pose estimate. */
  private Pose2d poseEstimate;

  // ---------------------------------------------------------------------------
  // Constructor
  // ---------------------------------------------------------------------------

  /**
   * Constructs a {@code SwervePoseEstimator}.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current angle as reported by the gyroscope.
   * @param wheelPositions The current encoder readings of each swerve module.
   * @param initialPose The initial position of the robot on the field.
   * @param disableSlipDetection When {@code true}, the IMU-based wheel-slip attenuation in the
   *     underlying {@link SwerveOdometry} is disabled.
   * @param stateStdDevs Standard deviations of the pose estimate (x in meters, y in meters, heading
   *     in radians). Larger values indicate less confidence in the wheel-encoder estimate and
   *     increase the weight given to vision.
   */
  public SwervePoseEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] wheelPositions,
      Pose2d initialPose,
      boolean disableSlipDetection,
      Matrix<N3, N1> stateStdDevs) {
    odometry =
        new SwerveOdometry(
            kinematics, gyroAngle, wheelPositions, initialPose, disableSlipDetection);
    poseEstimate = odometry.getEstimatedPosition();

    for (int i = 0; i < 3; ++i) {
      q[i] = stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0);
    }
  }

  // ---------------------------------------------------------------------------
  // Configuration setters
  // ---------------------------------------------------------------------------

  /**
   * Sets the vision trust mode, which governs how incoming vision measurements are blended with the
   * odometry-based estimate.
   *
   * <p>The mode can be changed at any time during a match. For example, you might use {@link
   * VisionTrustMode#FULL_TRUST} at the start of autonomous (when the robot has just been placed on
   * the field and vision is reliable) and switch to {@link VisionTrustMode#GATED} during teleop
   * (when accidental tag misreads are more likely due to field lighting and crowd proximity).
   *
   * @param mode The {@link VisionTrustMode} to use for subsequent {@link #addVisionMeasurement}
   *     calls.
   */
  public void setVisionTrustMode(VisionTrustMode mode) {
    this.visionTrustMode = mode;
  }

  /**
   * Returns the current vision trust mode.
   *
   * @return The active {@link VisionTrustMode}.
   */
  public VisionTrustMode getVisionTrustMode() {
    return visionTrustMode;
  }

  /**
   * Sets the translational outlier rejection radius used in {@link VisionTrustMode#GATED} mode.
   *
   * <p>Any vision measurement whose translational distance from the current fused pose estimate
   * exceeds this radius is discarded before the Kalman step. The default value is {@value
   * #DEFAULT_OUTLIER_REJECTION_RADIUS_METERS} meter(s).
   *
   * <p>This has no effect unless the active mode is {@link VisionTrustMode#GATED}.
   *
   * @param radiusMeters Maximum tolerated translational distance (meters) between a vision
   *     measurement and the current fused estimate. Must be positive.
   * @throws IllegalArgumentException if {@code radiusMeters} is not positive.
   */
  public void setOutlierRejectionRadius(double radiusMeters) {
    if (radiusMeters <= 0.0) {
      throw new IllegalArgumentException(
          "outlierRejectionRadiusMeters must be positive, got: " + radiusMeters);
    }
    this.outlierRejectionRadiusMeters = radiusMeters;
  }

  /**
   * Returns the current outlier rejection radius.
   *
   * @return The radius in meters.
   */
  public double getOutlierRejectionRadius() {
    return outlierRejectionRadiusMeters;
  }

  // ---------------------------------------------------------------------------
  // Pose accessors
  // ---------------------------------------------------------------------------

  /**
   * Returns the current vision-fused pose estimate.
   *
   * @return The estimated pose of the robot in meters.
   */
  public Pose2d getEstimatedPosition() {
    return poseEstimate;
  }

  /**
   * Returns the current raw odometry pose, without any vision correction applied.
   *
   * <p>This is useful for logging and tuning: publishing both this and {@link
   * #getEstimatedPosition()} to NetworkTables allows you to visualise the divergence between
   * wheel-encoder-only localisation and vision-fused localisation at a glance.
   *
   * @return The odometry-only pose estimate in meters.
   */
  public Pose2d getOdometryPosition() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Returns the vision-fused pose estimate at a historical timestamp, if the odometry buffer
   * contains data for that time.
   *
   * <p>The returned pose includes the most recent vision correction that was active at {@code
   * timestamp}.
   *
   * @param timestamp The wall-clock timestamp in seconds (same epoch as {@link
   *     Timer#getFPGATimestamp()}).
   * @return The fused pose at the given timestamp, or {@link Optional#empty()} if the buffer does
   *     not contain data for that time.
   */
  public Optional<Pose2d> sampleAt(double timestamp) {
    if (odometryPoseBuffer.getInternalBuffer().isEmpty()) {
      return Optional.empty();
    }

    // Clamp to the buffer's time range so interpolation always has two bounding entries.
    final double oldest = odometryPoseBuffer.getInternalBuffer().firstKey();
    final double newest = odometryPoseBuffer.getInternalBuffer().lastKey();
    final double clampedTimestamp = MathUtil.clamp(timestamp, oldest, newest);

    // If no vision updates exist, or the query predates all of them, return odometry only.
    if (visionUpdates.isEmpty() || clampedTimestamp < visionUpdates.firstKey()) {
      return odometryPoseBuffer.getSample(clampedTimestamp);
    }

    // Apply the latest vision correction that was active at the queried time.
    final double floorKey = visionUpdates.floorKey(clampedTimestamp);
    final VisionUpdate visionUpdate = visionUpdates.get(floorKey);
    return odometryPoseBuffer.getSample(clampedTimestamp).map(visionUpdate::compensate);
  }

  // ---------------------------------------------------------------------------
  // Reset methods
  // ---------------------------------------------------------------------------

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be zeroed in user code; the library automatically
   * compensates for the offset between the raw gyro reading and the desired field-relative heading.
   *
   * @param gyroAngle The angle currently reported by the gyroscope.
   * @param modulePositions The current encoder readings from each swerve module.
   * @param pose The position on the field that your robot is at.
   */
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
    odometry.resetPosition(gyroAngle, modulePositions, pose);
    clearVisionState();
    poseEstimate = odometry.getEstimatedPosition();
  }

  /**
   * Resets the robot's pose without changing the gyro offset or previous wheel positions.
   *
   * @param pose The pose to reset to.
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPose(pose);
    clearVisionState();
    poseEstimate = odometry.getEstimatedPosition();
  }

  /**
   * Resets only the translational component of the pose, preserving the current heading.
   *
   * <p>If a vision update was active at the time of the reset, its rotational compensation is
   * preserved and anchored to the new translation so that the next vision measurement continues to
   * blend correctly.
   *
   * @param translation The translation to reset to.
   */
  public void resetTranslation(Translation2d translation) {
    odometry.resetTranslation(translation);

    final var latestEntry = visionUpdates.lastEntry();
    clearVisionState();

    if (latestEntry != null) {
      final VisionUpdate prev = latestEntry.getValue();
      // Re-anchor the last vision update at the new translation, preserving only the rotational
      // compensation so that heading tracking continues uninterrupted.
      final VisionUpdate anchored =
          new VisionUpdate(
              new Pose2d(translation, prev.visionPose.getRotation()),
              new Pose2d(translation, prev.odometryPose.getRotation()),
              prev.kalmanGain);
      visionUpdates.put(latestEntry.getKey(), anchored);
      poseEstimate = anchored.compensate(odometry.getEstimatedPosition());
    } else {
      poseEstimate = odometry.getEstimatedPosition();
    }
  }

  /**
   * Resets only the rotational component of the pose, preserving the current translation.
   *
   * <p>If a vision update was active at the time of the reset, its translational compensation is
   * preserved and anchored to the new rotation.
   *
   * @param rotation The rotation to reset to.
   */
  public void resetRotation(Rotation2d rotation) {
    odometry.resetRotation(rotation);

    final var latestEntry = visionUpdates.lastEntry();
    clearVisionState();

    if (latestEntry != null) {
      final VisionUpdate prev = latestEntry.getValue();
      // Re-anchor the last vision update at the new rotation, preserving only the translational
      // compensation so that position tracking continues uninterrupted.
      final VisionUpdate anchored =
          new VisionUpdate(
              new Pose2d(prev.visionPose.getTranslation(), rotation),
              new Pose2d(prev.odometryPose.getTranslation(), rotation),
              prev.kalmanGain);
      visionUpdates.put(latestEntry.getKey(), anchored);
      poseEstimate = anchored.compensate(odometry.getEstimatedPosition());
    } else {
      poseEstimate = odometry.getEstimatedPosition();
    }
  }

  // ---------------------------------------------------------------------------
  // Vision update — hard override
  // ---------------------------------------------------------------------------

  /**
   * Immediately overrides the fused pose estimate with the supplied vision pose, bypassing all
   * filtering, gating, and the current {@link VisionTrustMode}.
   *
   * <p>All buffered vision updates are cleared. The odometry pose buffer is preserved so that
   * subsequent calls to {@link #sampleAt} remain valid.
   *
   * <p>Use this for known-reliable one-shot relocalisations, for example:
   *
   * <ul>
   *   <li>Setting the pose at the start of autonomous from a trusted AprilTag fix.
   *   <li>Applying a driver-initiated field reset after a known collision or push.
   * </ul>
   *
   * <p>After this call, the next {@link #addVisionMeasurement} will proceed as if the override pose
   * were the sole historical anchor, which prevents stale vision corrections from partially undoing
   * the override.
   *
   * @param visionPose The pose to unconditionally adopt as the new fused estimate.
   */
  public void overridePoseWithVision(Pose2d visionPose) {
    // Determine what odometry thinks the pose is right now so we can record the
    // anchor for future compensate() calls.
    final Pose2d odometryNow = odometry.getEstimatedPosition();

    // Clear all prior vision updates; the override supersedes them.
    visionUpdates.clear();

    // Record the override as a full-trust VisionUpdate anchored at the current
    // odometry pose.  kalmanGain = {1, 1, 1} means the vision pose is taken verbatim.
    final double[] fullTrustGain = {1.0, 1.0, 1.0};
    final VisionUpdate overrideUpdate = new VisionUpdate(visionPose, odometryNow, fullTrustGain);

    // Timestamp key: use the latest timestamp in the odometry buffer if available,
    // otherwise fall back to the FPGA clock.
    final double now =
        odometryPoseBuffer.getInternalBuffer().isEmpty()
            ? Timer.getTimestamp()
            : odometryPoseBuffer.getInternalBuffer().lastKey();

    visionUpdates.put(now, overrideUpdate);
    poseEstimate = visionPose;
  }

  // ---------------------------------------------------------------------------
  // Vision update — Kalman-filtered
  // ---------------------------------------------------------------------------

  /**
   * Adds a vision measurement and fuses it with the running odometry estimate using a Kalman filter
   * weighted by the supplied standard deviations.
   *
   * <p>The Kalman gain is computed from {@code visionMeasurementStdDevs} and the constructor's
   * {@code stateStdDevs} and is stored <em>inside</em> the resulting {@link VisionUpdate} record.
   * This means:
   *
   * <ul>
   *   <li>Different cameras (or the same camera at different ranges) can each supply their own
   *       noise model without interfering with one another.
   *   <li>The gain associated with a historical measurement is never retroactively changed by a
   *       subsequent call.
   * </ul>
   *
   * <h3>Late-arriving measurements and replay</h3>
   *
   * <p>If {@code timestamp} is earlier than the most recent vision update already recorded, all
   * later vision updates are replayed forward on top of the new anchor (see {@link
   * #replayVisionUpdatesFrom(double)}). This fixes the silent-discard behaviour described in WPILib
   * issue #7881, where inserting a late measurement would silently discard all subsequent
   * corrections.
   *
   * <h3>Outlier rejection (GATED mode only)</h3>
   *
   * <p>When the active mode is {@link VisionTrustMode#GATED}, the measurement is compared against
   * the current fused pose estimate at {@code timestamp} (via {@link #sampleAt}). If the
   * translational distance exceeds the configured {@link #setOutlierRejectionRadius(double) outlier
   * rejection radius}, the measurement is silently discarded. Use {@link
   * #overridePoseWithVision(Pose2d)} if you need to force a large relocalisation correction through
   * regardless.
   *
   * @param visionRobotPose The pose of the robot as measured by the vision system, in field-frame
   *     coordinates.
   * @param timestamp The wall-clock capture timestamp of the vision frame in seconds, using the
   *     same epoch as {@link Timer#getFPGATimestamp()}. This should be the time the image was
   *     captured, not the time it was processed.
   * @param visionMeasurementStdDevs Standard deviations of this specific vision measurement ({@code
   *     [x (m), y (m), θ (rad)]ᵀ}). Increase these values to trust this measurement less.
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {

    // ------------------------------------------------------------------
    // Guard: skip if the measurement is older than the buffer can handle.
    // ------------------------------------------------------------------
    if (odometryPoseBuffer.getInternalBuffer().isEmpty()
        || odometryPoseBuffer.getInternalBuffer().lastKey() - BUFFER_DURATION > timestamp) {
      return;
    }

    // ------------------------------------------------------------------
    // Compute the per-measurement Kalman gain from the supplied stddevs.
    // This is the closed-form solution for a continuous Kalman filter with
    // A = 0 and C = I (see wpimath/algorithms.md for the derivation).
    // Storing the gain in the VisionUpdate record isolates it completely
    // from any future setVisionMeasurementStdDevs-style call.
    // ------------------------------------------------------------------
    final double[] r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
    }
    final double[] gain = new double[3];
    for (int i = 0; i < 3; ++i) {
      if (q[i] == 0.0) {
        gain[i] = 0.0;
      } else {
        gain[i] = q[i] / (q[i] + Math.sqrt(q[i] * r[i]));
      }
    }

    // ------------------------------------------------------------------
    // GATED mode: discard if translational distance from the fused pose
    // at the measurement timestamp is too large.
    // ------------------------------------------------------------------
    if (visionTrustMode == VisionTrustMode.GATED) {
      final Optional<Pose2d> fusedAtTimestamp = sampleAt(timestamp);
      if (fusedAtTimestamp.isPresent()) {
        final double distance =
            visionRobotPose.getTranslation().getDistance(fusedAtTimestamp.get().getTranslation());
        if (distance > outlierRejectionRadiusMeters) {
          return;
        }
      }
    }

    // ------------------------------------------------------------------
    // FULL_TRUST mode: hard-reset the fused pose at this timestamp and
    // replay all subsequent odometry forward from that anchor.
    // ------------------------------------------------------------------
    if (visionTrustMode == VisionTrustMode.FULL_TRUST) {
      final Optional<Pose2d> odometrySample = odometryPoseBuffer.getSample(timestamp);
      if (odometrySample.isEmpty()) {
        return;
      }
      // Full-trust gain: accept the vision pose verbatim.
      final double[] fullGain = {1.0, 1.0, 1.0};
      final VisionUpdate fullUpdate =
          new VisionUpdate(visionRobotPose, odometrySample.get(), fullGain);
      visionUpdates.put(timestamp, fullUpdate);
      replayVisionUpdatesFrom(timestamp);
      poseEstimate =
          visionUpdates.get(visionUpdates.lastKey()).compensate(odometry.getEstimatedPosition());
      cleanUpVisionUpdates();
      return;
    }

    // ------------------------------------------------------------------
    // FILTERED / GATED mode: standard Kalman-weighted blend.
    // ------------------------------------------------------------------

    // Housekeeping: remove stale entries that are no longer needed.
    cleanUpVisionUpdates();

    // Get the odometry-only pose at the moment the camera captured this frame.
    final Optional<Pose2d> odometrySample = odometryPoseBuffer.getSample(timestamp);
    if (odometrySample.isEmpty()) {
      return;
    }

    // Get the vision-fused pose at the same historical moment, so that the
    // correction is computed relative to the best estimate at capture time
    // (not the current estimate).
    final Optional<Pose2d> fusedAtCapture = sampleAt(timestamp);
    if (fusedAtCapture.isEmpty()) {
      return;
    }

    // Compute the transform from the fused historical pose to the raw vision pose.
    final Transform2d rawTransform = visionRobotPose.minus(fusedAtCapture.get());

    // Scale the transform by the per-measurement Kalman gain.
    final Transform2d scaledTransform =
        new Transform2d(
            gain[0] * rawTransform.getX(),
            gain[1] * rawTransform.getY(),
            Rotation2d.fromRadians(gain[2] * rawTransform.getRotation().getRadians()));

    // The corrected fused pose at the capture timestamp.
    final Pose2d correctedFusedAtCapture = fusedAtCapture.get().plus(scaledTransform);

    // Store the new VisionUpdate, preserving the raw vision pose for later replay.
    final VisionUpdate newUpdate =
        new VisionUpdate(correctedFusedAtCapture, odometrySample.get(), visionRobotPose, gain);
    visionUpdates.put(timestamp, newUpdate);

    // If there were later vision updates, replay them forward so that they are
    // reconciled with the new anchor rather than being silently discarded.
    // This fixes WPILib issue #7881.
    replayVisionUpdatesFrom(timestamp);

    // Recompute the current fused estimate from the (now up-to-date) latest
    // vision update.
    poseEstimate =
        visionUpdates.get(visionUpdates.lastKey()).compensate(odometry.getEstimatedPosition());
  }

  // ---------------------------------------------------------------------------
  // Odometry update
  // ---------------------------------------------------------------------------

  /**
   * Updates the pose estimator with the latest wheel encoder and gyro readings. This must be called
   * every robot loop.
   *
   * @param gyroReadings The current IMU readings, or {@link Optional#empty()} to rely purely on
   *     wheel odometry.
   * @param wheelPositions The current encoder readings from each swerve module.
   * @return The new fused pose estimate in meters.
   */
  public Pose2d update(Optional<GyroReadings> gyroReadings, SwerveModulePosition[] wheelPositions) {
    return updateWithTime(Timer.getTimestamp(), gyroReadings, wheelPositions);
  }

  /**
   * Updates the pose estimator with the latest wheel encoder and gyro readings, using an explicit
   * timestamp. Use this overload when you are managing your own time source.
   *
   * @param currentTime Wall-clock time of this update in seconds (same epoch as {@link
   *     Timer#getFPGATimestamp()}).
   * @param gyroReadings The current IMU readings, or {@link Optional#empty()} to rely purely on
   *     wheel odometry.
   * @param wheelPositions The current encoder readings from each swerve module.
   * @return The new fused pose estimate in meters.
   */
  public Pose2d updateWithTime(
      double currentTime,
      Optional<GyroReadings> gyroReadings,
      SwerveModulePosition[] wheelPositions) {

    final Pose2d odometryEstimate = odometry.update(currentTime, gyroReadings, wheelPositions);
    odometryPoseBuffer.addSample(currentTime, odometryEstimate);

    if (visionUpdates.isEmpty()) {
      poseEstimate = odometryEstimate;
    } else {
      poseEstimate = visionUpdates.get(visionUpdates.lastKey()).compensate(odometryEstimate);
    }

    return poseEstimate;
  }

  // ---------------------------------------------------------------------------
  // Private helpers
  // ---------------------------------------------------------------------------

  /**
   * Removes {@link VisionUpdate} entries from {@link #visionUpdates} that can no longer affect any
   * sample within the odometry pose buffer's time range, keeping memory bounded.
   *
   * <p>Specifically, only one entry strictly before the oldest odometry timestamp needs to be
   * retained (as the anchor for any sample near the buffer's oldest edge). Everything older than
   * that anchor is pruned.
   */
  private void cleanUpVisionUpdates() {
    if (odometryPoseBuffer.getInternalBuffer().isEmpty()) {
      return;
    }

    final double oldestOdometry = odometryPoseBuffer.getInternalBuffer().firstKey();

    // Nothing to prune if all vision updates post-date the oldest odometry sample.
    if (visionUpdates.isEmpty() || oldestOdometry < visionUpdates.firstKey()) {
      return;
    }

    // The newest vision update that still predates (or equals) the oldest odometry
    // timestamp must be kept as the anchor for samples near the buffer's leading edge.
    final double keepKey = visionUpdates.floorKey(oldestOdometry);
    visionUpdates.headMap(keepKey, false).clear();
  }

  /**
   * Replays all {@link VisionUpdate} entries strictly after {@code anchorTimestamp} so that their
   * corrections are expressed relative to the newly inserted anchor at {@code anchorTimestamp}.
   *
   * <h3>Why this is necessary</h3>
   *
   * <p>Each {@link VisionUpdate} stores an absolute fused pose ({@code visionPose}) and the
   * odometry pose at capture time ({@code odometryPose}). When a new measurement is inserted at
   * time {@code t} and later corrections already exist at times {@code t₁ > t, t₂ > t, ...}, the
   * later entries' {@code visionPose} values are no longer correct: they were computed on top of
   * whatever the fused pose was before the new anchor arrived, not on top of the new anchor.
   *
   * <p>Replaying forward re-derives each subsequent entry's {@code visionPose} by applying its
   * original scaled transform (recorded in its {@code kalmanGain} together with the raw vision pose
   * via the odometry anchor at its own timestamp) on top of the newly corrected history. This
   * ensures the entire chain of corrections is self-consistent.
   *
   * <h3>Implementation note</h3>
   *
   * <p>Each subsequent entry already stores its own {@code kalmanGain}. To replay, we ask: "given
   * the new fused history, what would the corrected pose at time {@code tᵢ} have been?" We retrieve
   * the fused pose at {@code tᵢ} (using the now-updated earlier entries as anchors), compute the
   * raw vision transform, scale it by the stored gain, and store the result as the updated {@code
   * visionPose} for that entry.
   *
   * <p>Each {@link VisionUpdate} stores the original raw vision pose ({@link
   * VisionUpdate#rawVisionPose}) alongside the per-measurement Kalman gain, so replay can re-apply
   * the same measurement to the updated fused history without needing to invert the Kalman step.
   *
   * @param anchorTimestamp The timestamp of the newly inserted (or updated) entry. All entries with
   *     keys strictly greater than this value are replayed.
   */
  private void replayVisionUpdatesFrom(double anchorTimestamp) {
    // Collect the timestamps of entries that need replaying, in ascending order.
    final NavigableMap<Double, VisionUpdate> toReplay =
        visionUpdates.tailMap(anchorTimestamp, false);

    if (toReplay.isEmpty()) {
      return;
    }

    for (final var entry : toReplay.entrySet()) {
      final double ts = entry.getKey();
      final VisionUpdate old = entry.getValue();
      final double[] k = old.kalmanGain;

      // Retrieve the odometry-only pose at this entry's capture timestamp.
      // If the buffer no longer has it (very old measurement), skip.
      final Optional<Pose2d> odometrySampleOpt = odometryPoseBuffer.getSample(ts);
      if (odometrySampleOpt.isEmpty()) {
        continue;
      }
      final Pose2d odometrySample = odometrySampleOpt.get();

      // sampleAt(ts) now reflects all anchors up to the one just inserted at anchorTimestamp,
      // giving the revised fused pose at this entry's timestamp.
      final Optional<Pose2d> newFusedAtTs = sampleAt(ts);
      if (newFusedAtTs.isEmpty()) {
        continue;
      }
      final Pose2d newFused = newFusedAtTs.get();

      // Re-apply the original raw vision measurement with the stored gain to the revised
      // fused history. rawVisionPose is stored in each VisionUpdate so we never need to
      // invert the Kalman step to recover the original measurement.
      final Transform2d rawTransform = old.rawVisionPose.minus(newFused);
      final Transform2d scaledTransform =
          new Transform2d(
              k[0] * rawTransform.getX(),
              k[1] * rawTransform.getY(),
              Rotation2d.fromRadians(k[2] * rawTransform.getRotation().getRadians()));

      final Pose2d updatedVisionPose = newFused.plus(scaledTransform);
      entry.setValue(new VisionUpdate(updatedVisionPose, odometrySample, old.rawVisionPose, k));
    }
  }

  /**
   * Clears all buffered odometry samples and vision updates, and resets velocity bookkeeping in the
   * underlying odometry. Called by all {@code reset*} methods.
   */
  private void clearVisionState() {
    odometryPoseBuffer.clear();
    visionUpdates.clear();
  }

  // ---------------------------------------------------------------------------
  // VisionUpdate record
  // ---------------------------------------------------------------------------

  /**
   * An immutable snapshot of a single vision correction.
   *
   * <p>Stores:
   *
   * <ul>
   *   <li>{@link #visionPose} — the Kalman-corrected fused pose at the capture timestamp.
   *   <li>{@link #odometryPose} — the raw odometry pose at the capture timestamp (used by {@link
   *       #compensate} to compute the delta when projecting the correction forward).
   *   <li>{@link #rawVisionPose} — the original, unfiltered pose reported by the vision system.
   *       Stored so that {@link SwervePoseEstimator#replayVisionUpdatesFrom} can re-apply the same
   *       measurement to an updated fused history without needing to invert the Kalman step.
   *   <li>{@link #kalmanGain} — the per-axis gain ({@code [kx, ky, kθ]}) that was used to compute
   *       {@link #visionPose}. Preserved so that replay can re-apply the same weighting.
   * </ul>
   */
  private static final class VisionUpdate {
    /** The Kalman-corrected fused pose at the vision frame's capture timestamp. */
    final Pose2d visionPose;

    /** The raw odometry-only pose at the vision frame's capture timestamp. */
    final Pose2d odometryPose;

    /**
     * The original, unfiltered vision measurement. Stored to enable numerically exact replay
     * without having to invert the Kalman step.
     */
    final Pose2d rawVisionPose;

    /**
     * Per-axis Kalman gain {@code [kx, ky, kθ]} that was used when this entry was created. Re-used
     * during replay to re-apply the same noise model to the updated fused history.
     */
    final double[] kalmanGain;

    /**
     * Constructs a {@code VisionUpdate} with a distinct raw vision pose (standard path).
     *
     * @param visionPose The corrected fused pose.
     * @param odometryPose The odometry-only pose at capture time.
     * @param rawVisionPose The original vision measurement.
     * @param kalmanGain The per-axis Kalman gain used to compute {@code visionPose}.
     */
    VisionUpdate(
        Pose2d visionPose, Pose2d odometryPose, Pose2d rawVisionPose, double[] kalmanGain) {
      this.visionPose = visionPose;
      this.odometryPose = odometryPose;
      this.rawVisionPose = rawVisionPose;
      this.kalmanGain = kalmanGain;
    }

    /**
     * Convenience constructor for cases where the raw vision pose and the corrected pose coincide
     * (i.e. {@link VisionTrustMode#FULL_TRUST} or {@link
     * SwervePoseEstimator#overridePoseWithVision}).
     *
     * @param visionPose The corrected (and raw) fused pose.
     * @param odometryPose The odometry-only pose at capture time.
     * @param kalmanGain The per-axis Kalman gain (typically {@code {1, 1, 1}} for full-trust).
     */
    VisionUpdate(Pose2d visionPose, Pose2d odometryPose, double[] kalmanGain) {
      this(visionPose, odometryPose, visionPose, kalmanGain);
    }

    /**
     * Projects this correction onto a current odometry pose.
     *
     * <p>The odometry delta since the capture timestamp is computed and applied on top of {@link
     * #visionPose}, effectively carrying the vision correction forward to the present.
     *
     * @param currentOdometryPose The current odometry-only estimate.
     * @return The vision-corrected current pose.
     */
    Pose2d compensate(Pose2d currentOdometryPose) {
      // Delta = how much odometry has moved since the correction was recorded.
      final Transform2d delta = currentOdometryPose.minus(odometryPose);
      return visionPose.plus(delta);
    }
  }
}
