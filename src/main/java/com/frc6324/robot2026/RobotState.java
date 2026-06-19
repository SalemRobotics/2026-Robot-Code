package com.frc6324.robot2026;

import static com.frc6324.robot2026.subsystems.drive.DriveConstants.MODULE_TRANSLATIONS;

import com.frc6324.lib.odometry.GyroReadings;
import com.frc6324.lib.odometry.SwervePoseEstimator;
import com.frc6324.lib.util.FieldConstants;
import com.frc6324.lib.util.Lazy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private static final Lazy<RobotState> INSTANCE = new Lazy<>(RobotState::new);
  private static final Pose2d STARTING_POSE =
      new Pose2d(FieldConstants.FIELD_LENGTH / 2, FieldConstants.FIELD_WIDTH / 2, Rotation2d.kZero);

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
          RobotBase.isSimulation(),
          VecBuilder.fill(0.1, 0.1, 0.1));

  public void addOdometryObservation(
      double timestamp,
      SwerveModulePosition[] modulePositions,
      Optional<GyroReadings> gyroReadings) {
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
    if (!RobotBase.isReal()) {
      return;
    }

    poseEstimator.addVisionMeasurement(pose, timestamp, stddevs);
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

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(lastGyroAngle, lastModulePositions, pose);
  }
}
