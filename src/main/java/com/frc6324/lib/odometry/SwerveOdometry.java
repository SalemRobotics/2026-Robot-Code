package com.frc6324.lib.odometry;

import static edu.wpi.first.units.Units.Gs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.Units;
import java.util.Optional;

/**
 * IMU-enhanced swerve drive odometry.
 *
 * <p>Mirrors the public API of WPILib's {@code SwerveDriveOdometry} exactly, with the sole
 * exception that {@link #update} accepts an {@code Optional<GyroReadings>} and a {@code timestamp}
 * in place of a bare {@code Rotation2d}. When the optional is present, the IMU data is used to:
 *
 * <ol>
 *   <li>Supply a yaw reading rather than integrating wheel-encoder angular rates.
 *   <li>Correct the wheel-encoder translation deltas for the robot's pitch and roll so that
 *       distances reported by encoders are mapped to true ground-plane displacement.
 *   <li>Detect wheel slip by comparing IMU-derived linear acceleration against encoder-derived
 *       acceleration (computed accurately using the elapsed {@code dt}), and attenuate the
 *       contribution of all modules when slip is detected.
 * </ol>
 *
 * <p>When {@code Optional.empty()} is supplied the behaviour degrades gracefully to pure
 * wheel-encoder integration, identical to stock WPILib odometry.
 *
 * <h2>Slip detection</h2>
 *
 * <p>Each update, the encoder-derived chassis acceleration is computed as:
 *
 * <pre>
 *     a_enc = (v_current − v_previous) / dt
 * </pre>
 *
 * where {@code v} is the chassis-frame velocity derived from the raw wheel-position deltas and
 * {@code dt} is the elapsed time since the last call. This is compared against the IMU's measured
 * forward/lateral accelerations. If the magnitudes diverge by more than {@link
 * #SLIP_ACCEL_THRESHOLD_M_S2}, all module deltas are scaled down by {@link #SLIP_WEIGHT_REDUCTION}.
 *
 * <p>Note: because the IMU provides only two chassis-axis acceleration components, per-module slip
 * isolation is not possible here. All modules are attenuated equally when chassis-level slip is
 * detected. Per-module isolation would require independent per-wheel IMUs or a more involved
 * contact-force model.
 *
 * <h2>Tilt correction</h2>
 *
 * <p>When the robot drives on a sloped surface the wheel encoders report arc length along the
 * inclined plane, not displacement on the flat field. Pitch (rotation about the robot's Y-axis)
 * reduces effective forward travel and roll (rotation about the X-axis) reduces effective lateral
 * travel. The correction factors are {@code cos(pitch)} and {@code cos(roll)} respectively, applied
 * independently to {@code dx} and {@code dy} in the chassis frame.
 *
 * <p>This is an approximation: on a surface tilted along an arbitrary axis, both pitch and roll
 * couple into both translation components. For typical FRC ramp angles (≤15°) the cross-coupling
 * error is below 1%, so the independent approximation is acceptable.
 */
public class SwerveOdometry {

  // -------------------------------------------------------------------------
  // Tuneable constants
  // -------------------------------------------------------------------------

  /**
   * Acceleration divergence (m/s²) between IMU and encoder-derived values above which chassis-level
   * slip is declared and module deltas are attenuated. Increase to be more tolerant of transient
   * mismatches; decrease to react more aggressively.
   */
  private static final double SLIP_ACCEL_THRESHOLD_M_S2 = 3.0;

  /**
   * Scale factor applied to each module's distance delta when slip is detected. {@code 0.0} ignores
   * encoder readings entirely; {@code 1.0} disables attenuation.
   */
  private static final double SLIP_WEIGHT_REDUCTION = 0.1;

  // -------------------------------------------------------------------------
  // State
  // -------------------------------------------------------------------------

  private final SwerveDriveKinematics kinematics;
  private final SwerveModulePosition[] previousWheelPositions;
  private final boolean disableSlipDetection;

  private Pose2d currentPose;
  private Rotation2d gyroOffset;
  private Rotation2d previousAngle;

  /**
   * Chassis-frame velocity (m/s) from the previous update cycle, used for computing encoder-derived
   * acceleration in slip detection. Stored separately from the twist so that tilt correction (which
   * modifies dx/dy) does not contaminate the velocity history.
   */
  private double previousVxMetersPerSecond = 0.0;

  private double previousVyMetersPerSecond = 0.0;

  /**
   * Timestamp (seconds) of the last {@link #update} call. {@code Double.NaN} until first update.
   */
  private double previousTimestampSeconds = Double.NaN;

  // -------------------------------------------------------------------------
  // Constructors (mirrors WPILib SwerveDriveOdometry)
  // -------------------------------------------------------------------------

  /**
   * Constructs a {@code SwerveOdometry} object.
   *
   * @param kinematics The swerve drive kinematics for your drivetrain.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The wheel positions reported by each module.
   * @param initialPose The starting position of the robot on the field.
   */
  public SwerveOdometry(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] wheelPositions,
      Pose2d initialPose,
      boolean disableSlipDetection) {
    this.kinematics = kinematics;
    this.previousWheelPositions = kinematics.copy(wheelPositions);
    this.currentPose = initialPose;
    this.gyroOffset = gyroAngle.unaryMinus().rotateBy(initialPose.getRotation());
    this.previousAngle = initialPose.getRotation();
    this.disableSlipDetection = disableSlipDetection;
  }

  /**
   * Constructs a {@code SwerveOdometry} object with the default pose at the origin.
   *
   * @param kinematics The swerve drive kinematics for your drivetrain.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The wheel positions reported by each module.
   */
  public SwerveOdometry(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] wheelPositions,
      boolean disableSlipDetection) {
    this(kinematics, gyroAngle, wheelPositions, Pose2d.kZero, disableSlipDetection);
  }

  // -------------------------------------------------------------------------
  // Public API (mirrors WPILib Odometry / SwerveDriveOdometry)
  // -------------------------------------------------------------------------

  /**
   * Returns the position of the robot on the field.
   *
   * @return The pose of the robot (x and y are in meters).
   */
  public Pose2d getEstimatedPosition() {
    return currentPose;
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset in user code; the library automatically
   * compensates for the gyro offset.
   *
   * @param gyroAngle The angle currently reported by the gyroscope.
   * @param modulePositions The current encoder readings.
   * @param pose The position on the field that your robot is at.
   */
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
    currentPose = pose;
    previousAngle = pose.getRotation();
    gyroOffset = gyroAngle.unaryMinus().rotateBy(pose.getRotation());
    kinematics.copyInto(modulePositions, previousWheelPositions);
    previousVxMetersPerSecond = 0.0;
    previousVyMetersPerSecond = 0.0;
    previousTimestampSeconds = Double.NaN;
  }

  /**
   * Resets the pose without changing the gyro offset or previous wheel positions.
   *
   * @param poseMeters The pose to reset to.
   */
  public void resetPose(Pose2d poseMeters) {
    // Shift gyroOffset by the difference in heading so that the next raw yaw reading
    // still maps to the correct field-relative angle after the pose change.
    gyroOffset = gyroOffset.plus(poseMeters.getRotation().minus(currentPose.getRotation()));
    currentPose = poseMeters;
    previousAngle = poseMeters.getRotation();
  }

  /**
   * Resets only the translation component of the pose.
   *
   * @param translation The translation to reset to.
   */
  public void resetTranslation(Translation2d translation) {
    currentPose = new Pose2d(translation, currentPose.getRotation());
  }

  /**
   * Resets only the rotation component of the pose.
   *
   * @param rotation The rotation to reset to.
   */
  public void resetRotation(Rotation2d rotation) {
    gyroOffset = gyroOffset.plus(rotation.minus(currentPose.getRotation()));
    currentPose = new Pose2d(currentPose.getTranslation(), rotation);
    previousAngle = rotation;
  }

  // -------------------------------------------------------------------------
  // Core update
  // -------------------------------------------------------------------------

  /**
   * Updates the robot's position on the field using forward kinematics and integration of the pose
   *
   * <p>over time.
   *
   * <p>Dispatches to {@link #updateWithGyro} when {@code gyroReadings} is present, or
   *
   * <p>{@link #updateWithoutGyro} otherwise.
   *
   * @param timestampSeconds Wall-clock timestamp of this measurement in seconds (e.g. from {@code
   *     <p>Timer.getFPGATimestamp()}). Used to compute {@code dt} for accurate encoder-derived
   *     <p>acceleration in slip detection.
   * @param gyroReadings IMU readings for this update cycle, or {@link Optional#empty()} to fall
   *     <p>back to pure wheel-encoder integration.
   * @param wheelPositions The current encoder readings from each swerve module.
   * @return The new estimated pose of the robot.
   */
  public Pose2d update(
      double timestampSeconds,
      Optional<GyroReadings> gyroReadings,
      SwerveModulePosition[] wheelPositions) {

    return gyroReadings.isPresent()
        ? updateWithGyro(timestampSeconds, gyroReadings.get(), wheelPositions)
        : updateWithoutGyro(timestampSeconds, wheelPositions);
  }

  // -------------------------------------------------------------------------

  // Private update implementations

  // -------------------------------------------------------------------------

  /**
   * IMU-assisted update. Called by {@link #update} when gyro readings are available.
   *
   * <p>Performs, in order:
   *
   * <ol>
   *   <li>Elapsed-time ({@code dt}) computation for slip detection.
   *   <li>Field-relative yaw from the raw gyro reading and the stored offset.
   *   <li>Raw chassis-frame twist from wheel-position deltas.
   *   <li>Optional slip attenuation of the translational twist components.
   *   <li>Angular override: replaces the kinematic {@code dtheta} with the gyro-derived delta so
   *       <p>yaw integration errors do not accumulate.
   *   <li>Tilt correction: scales {@code dx} by {@code cos(pitch)} and {@code dy} by
   *       <p>{@code cos(roll)} to project encoder arc length onto the flat field plane.
   *   <li>Pose integration via {@link Pose2d#exp}.
   *   <li>Bookkeeping: velocity history (from the pre-tilt raw twist), wheel-position copy,
   *       <p>timestamp update.
   * </ol>
   *
   * @param timestampSeconds Wall-clock timestamp in seconds.
   * @param imu Unwrapped IMU readings for this cycle; never {@code null}.
   * @param wheelPositions The current encoder readings from each swerve module.
   * @return The updated pose.
   */
  private Pose2d updateWithGyro(
      double timestampSeconds, GyroReadings imu, SwerveModulePosition[] wheelPositions) {
    // ------------------------------------------------------------------
    // 1. Elapsed time — NaN on the first call; disables slip detection
    //    for that cycle only.
    // ------------------------------------------------------------------
    final double dt =
        Double.isNaN(previousTimestampSeconds)
            ? Double.NaN
            : timestampSeconds - previousTimestampSeconds;

    // ------------------------------------------------------------------
    // 2. Field-relative heading from the gyro yaw plus the stored offset.
    // ------------------------------------------------------------------
    final Rotation2d angle = imu.rotation().toRotation2d().rotateBy(gyroOffset);

    // ------------------------------------------------------------------
    // 3. Raw chassis-frame twist from wheel-position deltas.
    //    Captured here so step 8 can read it before the wheel positions
    //    are overwritten by copyInto.
    // ------------------------------------------------------------------
    final Twist2d rawTwist = kinematics.toTwist2d(previousWheelPositions, wheelPositions);

    // ------------------------------------------------------------------
    // 4. Slip attenuation (translational components only).
    // ------------------------------------------------------------------
    final Twist2d twist =
        (!disableSlipDetection && !Double.isNaN(dt) && dt > 0.0)
            ? applySlipAttenuation(rawTwist, imu, dt)
            : rawTwist;

    // ------------------------------------------------------------------
    // 5. Replace the kinematic dtheta with the gyro-derived heading delta
    //    so that yaw integration errors in the wheel kinematics don't
    //    accumulate over time.
    // ------------------------------------------------------------------
    twist.dtheta = angle.minus(previousAngle).getRadians();

    // ------------------------------------------------------------------
    // 6. Tilt correction: project encoder arc-length onto the flat field.
    //
    //    WPILib Rotation3d convention: X = roll, Y = pitch, Z = yaw.
    //    Pitch (nose up/down) shrinks effective forward (dx) travel;
    //    roll (lean left/right) shrinks effective lateral (dy) travel.
    //    Independent-axis approximation; cross-coupling is negligible for
    //    FRC ramp angles (≤15°, cross-coupling error < 1%).
    // ------------------------------------------------------------------

    final double cosPitch = Math.cos(imu.rotation().getY());
    final double cosRoll = Math.cos(imu.rotation().getX());

    twist.dx *= cosPitch;
    twist.dy *= cosRoll;

    // ------------------------------------------------------------------
    // 7. Integrate the twist into the current pose.
    // ------------------------------------------------------------------
    currentPose = currentPose.exp(twist);

    // ------------------------------------------------------------------
    // 8. Bookkeeping.
    //
    //    Velocity history uses rawTwist (pre-tilt, pre-attenuation) so
    //    that the next cycle's slip detector compares like-for-like with
    //    the IMU's body-frame acceleration. It is read here, before
    //    copyInto overwrites previousWheelPositions (which would otherwise
    //    cause toTwist2d to return zero on a second call with the same
    //    positions).
    // ------------------------------------------------------------------

    previousAngle = angle;

    if (!Double.isNaN(dt) && dt > 0.0) {
      previousVxMetersPerSecond = rawTwist.dx / dt;
      previousVyMetersPerSecond = rawTwist.dy / dt;
    }

    kinematics.copyInto(wheelPositions, previousWheelPositions);
    previousTimestampSeconds = timestampSeconds;

    return currentPose;
  }

  /**
   * Wheel-encoder-only update. Called by {@link #update} when no gyro readings are available.
   *
   * <p>Behaviour is identical to stock WPILib {@code SwerveDriveOdometry}: the chassis-frame
   *
   * <p>twist is integrated directly, with {@code dtheta} supplying the heading change from the
   *
   * <p>kinematic model. No slip detection or tilt correction is applied.
   *
   * @param timestampSeconds Wall-clock timestamp in seconds.
   * @param wheelPositions The current encoder readings from each swerve module.
   * @return The updated pose.
   */
  private Pose2d updateWithoutGyro(double timestampSeconds, SwerveModulePosition[] wheelPositions) {
    // ------------------------------------------------------------------
    // 1. Raw chassis-frame twist from wheel-position deltas.
    // ------------------------------------------------------------------
    final Twist2d twist = kinematics.toTwist2d(previousWheelPositions, wheelPositions);

    // ------------------------------------------------------------------
    // 2. Advance the heading by integrating the kinematic angular rate.
    // ------------------------------------------------------------------
    final Rotation2d finalAngle = previousAngle.plus(Rotation2d.fromRadians(twist.dtheta));

    // ------------------------------------------------------------------
    // 3. Integrate the twist into the current pose.
    // ------------------------------------------------------------------
    currentPose = currentPose.exp(twist);

    // ------------------------------------------------------------------
    // 4. Bookkeeping.
    // ------------------------------------------------------------------
    previousAngle = finalAngle;
    kinematics.copyInto(wheelPositions, previousWheelPositions);
    previousTimestampSeconds = timestampSeconds;

    return currentPose;
  }

  // -------------------------------------------------------------------------
  // Private helpers
  // -------------------------------------------------------------------------

  /**
   * Returns a version of {@code rawTwist} whose translational components ({@code dx}, {@code dy})
   * are attenuated when chassis-level wheel slip is detected, leaving {@code dtheta} untouched
   * (heading comes from the gyro, not the wheels).
   *
   * <h3>Slip criterion</h3>
   *
   * <p>Encoder-derived chassis acceleration is:
   *
   * <pre>
   *     a_enc_x = (dx / dt − v_prev_x) / dt
   *     a_enc_y = (dy / dt − v_prev_y) / dt
   * </pre>
   *
   * If the magnitude of {@code (a_enc − a_imu)} exceeds {@link #SLIP_ACCEL_THRESHOLD_M_S2}, all
   * module deltas are scaled by {@link #SLIP_WEIGHT_REDUCTION}.
   *
   * <h3>Per-module limitation</h3>
   *
   * <p>The IMU gives a single chassis-axis acceleration vector, so it cannot isolate which
   * individual wheel is slipping. Attenuation is therefore applied uniformly across all modules.
   * Per-module isolation would require per-wheel force sensing or a more complex observer.
   *
   * @param rawTwist The un-attenuated chassis-frame twist for this cycle.
   * @param imu The IMU readings for this cycle.
   * @param dt Elapsed time since the previous update, in seconds. Must be {@code > 0}.
   * @return The (possibly attenuated) twist.
   */
  private Twist2d applySlipAttenuation(Twist2d rawTwist, GyroReadings imu, double dt) {
    // Encoder-derived velocity this cycle (m/s).
    final double encVx = rawTwist.dx / dt;
    final double encVy = rawTwist.dy / dt;

    // Encoder-derived acceleration (m/s²) = change in velocity over dt.
    final double encAccelX = (encVx - previousVxMetersPerSecond) / dt;
    final double encAccelY = (encVy - previousVyMetersPerSecond) / dt;

    double pitch = imu.rotation().getY(); // nose-up positive
    double roll = imu.rotation().getX(); // left-lean positive

    // IMU acceleration in robot-frame X (forward) and Y (left).
    final double imuAccelX =
        imu.accelerationX()
            .minus(Gs.one().times(Math.sin(pitch)))
            .in(Units.MetersPerSecondPerSecond);

    final double imuAccelY =
        imu.accelerationY()
            .minus(Gs.one().times(Math.sin(roll)))
            .in(Units.MetersPerSecondPerSecond);

    // Euclidean distance between IMU and encoder acceleration vectors.
    final double diffX = encAccelX - imuAccelX;
    final double diffY = encAccelY - imuAccelY;
    final double accelDivergence = Math.sqrt(diffX * diffX + diffY * diffY);

    if (accelDivergence <= SLIP_ACCEL_THRESHOLD_M_S2) {
      return rawTwist; // No slip detected.
    }

    // Linearly ramp attenuation between threshold and 2× threshold
    final double slipRatio =
        MathUtil.clamp(
            (accelDivergence - SLIP_ACCEL_THRESHOLD_M_S2) / SLIP_ACCEL_THRESHOLD_M_S2, 0.0, 1.0);
    final double weight = 1.0 - slipRatio * (1.0 - SLIP_WEIGHT_REDUCTION);

    return new Twist2d(rawTwist.dx * weight, rawTwist.dy * weight, rawTwist.dtheta);
  }
}
