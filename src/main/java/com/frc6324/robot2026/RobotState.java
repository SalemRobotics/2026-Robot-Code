package com.frc6324.robot2026;

import static com.frc6324.robot2026.subsystems.drive.DriveConstants.MODULE_TRANSLATIONS;

import com.frc6324.lib.odometry.GyroReadings;
import com.frc6324.lib.odometry.SwervePoseEstimator;
import com.frc6324.lib.util.FieldConstants;
import com.frc6324.lib.util.Lazy;
import com.frc6324.robot2026.Constants.Mode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private static final Lazy<RobotState> INSTANCE = new Lazy<>(RobotState::new);
  private static final Pose2d STARTING_POSE =
      new Pose2d(FieldConstants.FIELD_LENGTH / 2, FieldConstants.FIELD_WIDTH / 2, Rotation2d.kZero);
  private static final double ODOMETRY_STDDEV = 0.1;
  private static final double DRIFT_PER_METER = 0.05;

  private double driftStddev = 0;

  private SwerveModulePosition[] lastModulePositions = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };

  private SwerveModuleState[] lastModuleStates = {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()
  };

  private Rotation2d lastGyroAngle = Rotation2d.kZero;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

  private final SwervePoseEstimator poseEstimator =
      new SwervePoseEstimator(
          kinematics,
          Rotation2d.kZero,
          lastModulePositions,
          STARTING_POSE,
          // Constants.CURRENT_MODE == Mode.SIM,
          true,
          VecBuilder.fill(ODOMETRY_STDDEV, ODOMETRY_STDDEV, 0.1));

  public void addOdometryObservation(
      double timestamp,
      SwerveModulePosition[] modulePositions,
      Optional<GyroReadings> gyroReadings) {
    final Twist2d twist = kinematics.toTwist2d(lastModulePositions, modulePositions);
    driftStddev += DRIFT_PER_METER * Math.hypot(twist.dx, twist.dy);

    poseEstimator.updateWithTime(timestamp, gyroReadings, modulePositions);
    lastModulePositions = modulePositions;
    lastGyroAngle =
        gyroReadings.isPresent()
            ? gyroReadings.get().rotation().toRotation2d()
            : poseEstimator.getOdometryPosition().getRotation();
  }

  public void addStateObservation(SwerveModuleState[] moduleStates) {
    lastModuleStates = moduleStates;
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stddevs) {
    // Simulated cameras derive their measurements from this estimator's own pose, so fusing them
    // would feed the estimate back into itself. Replay still applies the logged measurements.
    if (Constants.CURRENT_MODE == Mode.SIM) {
      return;
    }

    final double odometryStddev = ODOMETRY_STDDEV + driftStddev;
    final double visionStddev = stddevs.get(0, 0);
    final double trust = ODOMETRY_STDDEV / odometryStddev;

    driftStddev =
        Math.max(
            0, odometryStddev * visionStddev / (odometryStddev + visionStddev) - ODOMETRY_STDDEV);

    poseEstimator.addVisionMeasurement(
        pose,
        timestamp,
        VecBuilder.fill(visionStddev * trust, stddevs.get(1, 0) * trust, stddevs.get(2, 0)));
  }

  @AutoLogOutput(key = "Odometry/DriftStddev")
  public double getDriftStddev() {
    return driftStddev;
  }

  @AutoLogOutput(key = "Odometry/Speeds")
  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(lastModuleStates);
  }

  /**
   * Gets the global {@link RobotState} instance.
   *
   * @return The global robot state.
   */
  public static RobotState getInstance() {
    return INSTANCE.get();
  }

  /**
   * Gets the current estimated position of the robot.
   *
   * @return The believed position of the robot.
   */
  @AutoLogOutput(key = "Odometry/Position")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * @param timestamp The timestamp at which to get
   * @return The pose at the given timestamp, or {@link Optional#empty()} if no odometry data is
   *     available at the timestamp.
   */
  public Optional<Pose2d> samplePoseAt(double timestamp) {
    return poseEstimator.sampleAt(timestamp);
  }

  /**
   * Gets the robot's chassis speeds at a historical timestamp.
   *
   * <p>Vision rules should use this instead of {@link #getSpeeds()} so that they evaluate the
   * robot's motion at the instant a frame was captured, rather than at the time the frame was
   * processed.
   *
   * @param timestamp The timestamp at which to get the speeds.
   * @return The chassis speeds at the given timestamp, or {@link Optional#empty()} if no speed data
   *     is available at the timestamp.
   */
  public Optional<ChassisSpeeds> sampleSpeedsAt(double timestamp) {
    return poseEstimator.sampleSpeedsAt(timestamp);
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(lastGyroAngle, lastModulePositions, pose);
    driftStddev = 0;
  }
}
