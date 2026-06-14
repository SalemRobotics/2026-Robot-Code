package com.frc6324.robot2026.commands;

import static edu.wpi.first.units.Units.Degrees;

import com.frc6324.lib.UninstantiableClass;
import com.frc6324.lib.util.AllianceFlipUtil;
import com.frc6324.lib.util.PoseExtensions;
import com.frc6324.lib.util.logging.LoggedTunableProfiledPID;
import com.frc6324.robot2026.subsystems.drive.Drive;
import com.frc6324.robot2026.subsystems.drive.DriveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.experimental.ExtensionMethod;
import org.jetbrains.annotations.Contract;
import org.littletonrobotics.junction.Logger;

@UninstantiableClass
@ExtensionMethod(PoseExtensions.class)
public final class DriveCommands {
  public static final double ANGLE_TOLERANCE = 2.5;
  public static final double POSITION_TOLERANCE = Units.inchesToMeters(2);
  public static final double TRANSLATION_MAX_VELOCITY =
      DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SEC;
  public static final double TRANSLATION_MAX_ACCELERATION = 7;
  public static final double ANGLE_MAX_VELOCITY = DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SEC;
  public static final double ANGLE_MAX_ACCELERATION = Units.degreesToRadians(720);
  public static final double CONTROLLER_INPUT_EXP = 2;
  public static final double DEADBAND = 0.1;

  public static final double POINTING_KP = 10;
  public static final double POINTING_KI = 0;
  public static final double POINTING_KD = 0.02;
  public static final Angle POINTING_TOLERANCE = Degrees.of(1);

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
    final Rotation2d direction = new Rotation2d(Math.atan2(y, x));

    // Return a translation of the given magnitude following the calculated
    // direction
    return new Translation2d(magnitude, direction);
  }

  public static Translation2d getLinearVelocityFromJoysticks(XboxController controller) {
    return getLinearVelocityFromJoysticks(-controller.getLeftY(), -controller.getLeftX());
  }

  /**
   * A regular field-relative arcade drive command.
   *
   * @param drive The drivetrain to command.
   * @param controller The controller to get input from.
   * @return The drive command.
   */
  public static Command joystickDrive(final Drive drive, final XboxController controller) {
    return drive.run(
        () -> {
          // Get the linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(controller);
          // Multiply the linear velocity by the drivetrain's max speed
          linearVelocity = linearVelocity.times(DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SEC);
          // Rotate the linear velocity to map to the current alliance
          if (AllianceFlipUtil.shouldFlip()) {
            linearVelocity = linearVelocity.unaryMinus();
          }

          // Get the rotational input
          double omega = -controller.getRightX();
          // Apply the deadband to the rotational input
          omega = MathUtil.applyDeadband(omega, DEADBAND);
          // Raise the rotational input to a higher power for finer control
          omega = Math.copySign(Math.pow(omega, CONTROLLER_INPUT_EXP), omega);
          // Multiply rotational input by max speed
          omega *= DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SEC;

          // Log the controller values
          Logger.recordOutput("DriveCommands/JoystickDrive/TranslationVector", linearVelocity);
          Logger.recordOutput("DriveCommands/JoystickDrive/RotationalRate", omega);

          // Send the request to the drivetrain
          drive.runFieldRelative(
              new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega));
        });
  }

  /**
   * @param name The name of the PID controller (used for dashboard inputs)
   * @return
   */
  public static LoggedTunableProfiledPID makeHeadingController(String name) {
    return new LoggedTunableProfiledPID(
        name, POINTING_KP, POINTING_KI, POINTING_KD, ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION);
  }
}
