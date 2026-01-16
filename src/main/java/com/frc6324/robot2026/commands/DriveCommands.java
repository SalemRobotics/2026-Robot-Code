package com.frc6324.robot2026.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.lib.util.PoseExtensions;
import com.frc6324.robot2026.subsystems.drive.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import lombok.experimental.ExtensionMethod;
import org.jetbrains.annotations.Contract;
import org.littletonrobotics.junction.Logger;

@UninstantiableClass
@ExtensionMethod(PoseExtensions.class)
public final class DriveCommands {
  static final double ANGLE_TOLERANCE = 2.5;
  static final double POSITION_TOLERANCE = Units.inchesToMeters(2);
  static final double TRANSLATION_MAX_VELOCITY = SwerveDrive.getMaxLinearSpeed();
  static final double TRANSLATION_MAX_ACCELERATION = 7;
  static final double ANGLE_MAX_VELOCITY = SwerveDrive.getMaxAngularSpeed();
  static final double ANGLE_MAX_ACCELERATION = Units.degreesToRadians(720);
  static final double CONTROLLER_INPUT_EXP = 2;
  static final double DEADBAND = 0.1;

  @Contract(value = "_ -> fail", pure = true)
  private DriveCommands() {
    throw new IllegalAccessError();
  }

  /**
   * Calculates the linear velocity the drivetrain should follow given joystick values.
   *
   * @param x The X input.
   * @param y The Y input.
   * @return The translation vector to follow.
   */
  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply the deadband to the total magnitude
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    // Raise magnitude to a higher power for finer control
    magnitude = Math.pow(magnitude, CONTROLLER_INPUT_EXP);

    // Calculate the direction in which to drive
    // The use of arctan to calculate direction can be shown by this graph:
    // https://www.desmos.com/calculator/5dbrh321dh
    Rotation2d direction = new Rotation2d(Math.atan2(y, x));

    // Return a translation of the given magnitude following the calculated
    // direction
    return new Translation2d(magnitude, direction);
  }

  public static Command driveToPose(SwerveDrive drive, Pose2d pose) {
    FollowPath.Builder builder = drive.getBLineBuilder();

    return drive.defer(
        () -> {
          Path.Waypoint target = new Path.Waypoint(pose, true);
          Path path = new Path(target);

          return builder.build(path);
        });
  }

  public static Command drivePointingTowards(
      SwerveDrive drive, XboxController controller, Pose2d target) {
    SwerveRequest.FieldCentricFacingAngle request =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveDrive.DRIVE_REQUEST)
            .withSteerRequestType(SwerveDrive.STEER_REQUEST)
            .withDesaturateWheelSpeeds(true);

    return drive.run(
        () -> {
          // Get the linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(-controller.getLeftY(), -controller.getLeftX());
          // Multiply the linear velocity by the drivetrain's max speed
          linearVelocity = linearVelocity.times(SwerveDrive.getMaxLinearSpeed());

          // Calculate the translation difference to the target
          Translation2d diff = target.getTranslation().minus(drive.getPose().getTranslation());
          // Calculate the angle that delta needs
          Rotation2d facing = new Rotation2d(diff.getX(), diff.getY());

          // Send the request to the drivetrain
          drive.setControl(
              request
                  .withVelocityX(linearVelocity.getX())
                  .withVelocityY(linearVelocity.getY())
                  .withTargetDirection(facing));
        });
  }

  /**
   * A regular field-relative arcade drive command.
   *
   * @param drive The drivetrain to command.
   * @param controller The controller to get input from.
   * @return The drive command.
   */
  public static Command joystickDrive(SwerveDrive drive, XboxController controller) {
    SwerveRequest.FieldCentric request =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveDrive.DRIVE_REQUEST)
            .withSteerRequestType(SwerveDrive.STEER_REQUEST)
            .withDesaturateWheelSpeeds(true);

    return drive.run(
        () -> {
          // Get the linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(-controller.getLeftY(), -controller.getLeftX());
          // Multiply the linear velocity by the drivetrain's max speed
          linearVelocity = linearVelocity.times(SwerveDrive.getMaxLinearSpeed());

          // Get the rotational input
          double omega = -controller.getRightX();
          // Apply the deadband to the rotational input
          omega = MathUtil.applyDeadband(omega, DEADBAND);
          // Raise the rotational input to a higher power for finer control
          omega = Math.pow(omega, CONTROLLER_INPUT_EXP);
          // Multiply rotational input by max speed
          omega *= SwerveDrive.getMaxAngularSpeed();

          // Log the controller values
          Logger.recordOutput("DriveCommands/JoystickDrive/TranslationVector", linearVelocity);
          Logger.recordOutput("DriveCommands/JoystickDrive/RotationalRate", omega);

          // Send the request to the drivetrain
          drive.setControl(
              request
                  .withVelocityX(linearVelocity.getX())
                  .withVelocityY(linearVelocity.getY())
                  .withRotationalRate(omega));
        });
  }
}
