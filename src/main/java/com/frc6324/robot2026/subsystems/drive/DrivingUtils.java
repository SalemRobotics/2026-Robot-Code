package com.frc6324.robot2026.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.Elastic;
import com.frc6324.lib.Elastic.Notification;
import com.frc6324.lib.Elastic.NotificationLevel;
import com.frc6324.lib.UninstantiableClass;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;

@UninstantiableClass
public final class DrivingUtils {
  private DrivingUtils() {
    throw new IllegalAccessError();
  }

  /** The maximum total tilt the robot can experience. */
  private static final double MAX_ACCEPTABLE_TILT = 12.5;

  /** The threshold at which to warn the driver. */
  private static final double TILT_WARNING_THRESHOLD = 7.0;

  /** The threshold under which to ignore lookahead. */
  private static final double TILT_LOOKAHEAD_THRESHOLD = 3.0;

  /** The amount of time to look into the future. */
  private static final double LOOKAHEAD_TIME = 0.5;

  /** The constant force of gravity (in m/s²) */
  private static final double GRAVITY_CONSTANT = 9.81;

  /**
   * The amount of consideration to use when guessing the robot's future tilt using lateral
   * acceleration
   */
  private static final double ACCELERATION_WEIGHT = 0.3;

  /**
   * The amount of consideration to use when guessing the robot's future tilt using angular
   * velocities
   */
  private static final double VELOCITY_WEIGHT = 0.7;

  /** The notification to send to the dashboard if the robot is projected to tilt. */
  private static final Notification TILT_NOTIFICATION =
      new Notification()
          .withLevel(NotificationLevel.WARNING)
          .withDisplaySeconds(5)
          .withTitle("Robot tilt detected!")
          .withDescription("Go easy on the sticks! The robot is on course to tilt.");

  private static double rollDegrees = 0;
  private static double pitchDegrees = 0;
  private static double rollVelocityDPS = 0;
  private static double pitchVelocityDPS = 0;
  private static double accelXMPSsq = 0;
  private static double accelYMPSsq = 0;
  private static Pose2d currentPose = Pose2d.kZero;
  private static ChassisSpeeds currentSpeeds = new ChassisSpeeds();

  /**
   * Updates the data held by the driving safety from the robot's IMU.
   *
   * @param roll The roll (rotation about the X axis) experienced by the robot.
   * @param pitch The pitch (rotation about the Y axis) experienced by the robot.
   * @param rollVelocity The rate at which the aforementioned roll is changing.
   * @param pitchVelocity The rate at which the aforemantioned pitch is changing.
   * @param accelX The robot-relative acceleration on its X axis.
   * @param accelY The robot-relative acceleration on its Y axis.
   */
  static void updateTilt(
      Angle roll,
      Angle pitch,
      AngularVelocity rollVelocity,
      AngularVelocity pitchVelocity,
      LinearAcceleration accelX,
      LinearAcceleration accelY) {
    rollDegrees = roll.in(Degrees);
    pitchDegrees = pitch.in(Degrees);
    rollVelocityDPS = rollVelocity.in(DegreesPerSecond);
    pitchVelocityDPS = pitchVelocity.in(DegreesPerSecond);
    accelXMPSsq = accelX.in(MetersPerSecondPerSecond);
    accelYMPSsq = accelY.in(MetersPerSecondPerSecond);
  }

  /**
   * Checks if the robot is going to tip over in the forseeable future. This is not 100% accurate
   * (as it cannot take into account the robot colliding with another object) and shouldn't be used
   * to majorly affect the robot.
   *
   * @return Whether or not the robot should limit its speed because it is going to tip over.
   */
  public static boolean shouldLimitMotion() {
    // Calculate the magnitude of the known tilt
    double tiltMagnitude = Math.hypot(rollDegrees, pitchDegrees);

    if (tiltMagnitude < TILT_LOOKAHEAD_THRESHOLD) {
      // No need to look ahead, current tilt is fine
      return false;
    }

    // Predict the robot's future tilt
    double rollLookahead = predictRoll();
    double pitchLookahead = predictPitch();

    // Calculate the magnitude of the future tilt
    double lookeaheadMagnitude = Math.hypot(rollLookahead, pitchLookahead);

    if (lookeaheadMagnitude > MAX_ACCEPTABLE_TILT) {
      // Return the tilt being at too great
      return true;
    } else if (lookeaheadMagnitude > TILT_WARNING_THRESHOLD) {
      // Send the tilt notification
      Elastic.sendNotification(TILT_NOTIFICATION);
    }

    // No problems!
    return false;
  }

  private static double predictRoll() {
    // Use velocity & acceleration to predict where roll will be
    double velocityRoll = rollDegrees + rollVelocityDPS * LOOKAHEAD_TIME;
    double accelerationRoll =
        rollDegrees + Math.toDegrees(Math.atan2(accelYMPSsq, GRAVITY_CONSTANT)) * LOOKAHEAD_TIME;

    // Return the weighted roll
    return (velocityRoll * VELOCITY_WEIGHT) + (accelerationRoll * ACCELERATION_WEIGHT);
  }

  private static double predictPitch() {
    // Use velocity & acceleration to predict where pitch will be
    double velocityPitch = pitchDegrees + pitchVelocityDPS * LOOKAHEAD_TIME;
    double accelerationPitch =
        pitchDegrees + Math.toDegrees(Math.atan2(accelXMPSsq, GRAVITY_CONSTANT)) * LOOKAHEAD_TIME;

    // Return the weighted pitch
    return (velocityPitch * VELOCITY_WEIGHT) + (accelerationPitch * ACCELERATION_WEIGHT);
  }

  /**
   * Updates the recognized odometry of the robot.
   *
   * @param currentPose The robot's last known pose.
   * @param currentSpeedsRobotRelative The robot's last known speeds.
   */
  public static void updateOdometry(Pose2d currentPose, ChassisSpeeds currentSpeedsRobotRelative) {
    DrivingUtils.currentPose = currentPose;
    DrivingUtils.currentSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            currentSpeedsRobotRelative, currentPose.getRotation());
  }

  /**
   * Estimates the future pose of the robot.
   *
   * @param lookaheadPeriod How many seconds in the future to estimate.
   * @return The pose of the robot in {@code lookaheadPeriod} seconds.
   */
  public static Pose2d estimateFuturePose(double lookaheadPeriod) {
    Translation2d translationRate =
        new Translation2d(
            currentSpeeds.vxMetersPerSecond + accelXMPSsq * lookaheadPeriod,
            currentSpeeds.vyMetersPerSecond + accelYMPSsq * lookaheadPeriod);
    Translation2d translation =
        currentPose.getTranslation().plus(translationRate.times(lookaheadPeriod));

    Rotation2d rotationalRate = new Rotation2d(currentSpeeds.omegaRadiansPerSecond);
    Rotation2d rotation = currentPose.getRotation().plus(rotationalRate.times(lookaheadPeriod));

    return new Pose2d(translation, rotation);
  }
}
