package com.frc6324.robot2026;

import static com.frc6324.robot2026.subsystems.drive.DriveConstants.MODULE_TRANSLATIONS;

import com.frc6324.lib.util.FieldConstants;
import com.frc6324.lib.util.Lazy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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

  private Rotation2d rawGyroAngle = STARTING_POSE.getRotation();

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics,
          rawGyroAngle,
          lastModulePositions,
          // start odometry in the middle of the field so that it is a consistent distance
          // from a starting position on the blue or red alliance.
          STARTING_POSE);

  private RobotState() {
    // AutoLogOutputManager.addObject(this);
  }

  public void addOdometryObservation(
      double timestamp, SwerveModulePosition[] modulePositions, Optional<Rotation2d> gyroAngle) {
    final Rotation2d rotation =
        gyroAngle.orElseGet(
            () -> {
              final Twist2d twist = kinematics.toTwist2d(lastModulePositions, modulePositions);
              return rawGyroAngle.plus(new Rotation2d(twist.dtheta));
            });

    rawGyroAngle = rotation;
    poseEstimator.updateWithTime(timestamp, rotation, modulePositions);
    lastModulePositions = modulePositions;
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

  public static RobotState getInstance() {
    return INSTANCE.get();
  }

  @AutoLogOutput(key = "Odometry/Position")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public Optional<Pose2d> samplePoseAt(double timestamp) {
    return poseEstimator.sampleAt(timestamp);
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroAngle, lastModulePositions, pose);
  }
}
