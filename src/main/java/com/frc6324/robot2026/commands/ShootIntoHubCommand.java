package com.frc6324.robot2026.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.frc6324.lib.util.FieldConstants;
import com.frc6324.robot2026.subsystems.drive.SwerveDrive;
import com.frc6324.robot2026.subsystems.indexer.Indexer;
import com.frc6324.robot2026.subsystems.shooter.Shooter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** A command that forces the drivetrain to point at the hub and shoot into it. */
public class ShootIntoHubCommand extends Command {
  private final SwerveDrive drive;
  private final XboxController controller;
  private final Shooter shooter;
  private final Indexer indexer;

  private Translation2d hubTranslation = Translation2d.kZero;
  private boolean commandedFeederWheel = false;

  private final SwerveRequest.FieldCentricFacingAngle request =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveDrive.DRIVE_REQUEST)
          .withSteerRequestType(SwerveDrive.STEER_REQUEST)
          .withHeadingPID(DriveCommands.POINTING_KP, 0, DriveCommands.POINTING_KD)
          .withDesaturateWheelSpeeds(true);

  /**
   * Creates a new hub shot command.
   *
   * @param drive The drivetrain to command.
   * @param controller The controller providing X/Y velocity inputs.
   * @param shooter The shooter being commanded.
   * @param indexer The indexer connected to the shooter.
   */
  public ShootIntoHubCommand(
      SwerveDrive drive, CommandXboxController controller, Shooter shooter, Indexer indexer) {
    this.drive = drive;
    this.controller = controller.getHID();
    this.shooter = shooter;
    this.indexer = indexer;
  }

  @Override
  public void initialize() {
    hubTranslation = FieldConstants.getAllianceHub().getTranslation();
    commandedFeederWheel = false;

    indexer.runIndexerWheel();
  }

  @Override
  public void execute() {
    // Get the linear velocity
    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            -controller.getLeftY(), -controller.getLeftX());
    // Multiply the linear velocity by the drivetrain's max speed
    linearVelocity = linearVelocity.times(SwerveDrive.getMaxLinearSpeed());

    // Calculate the translation difference to the target
    final Translation2d diff = hubTranslation.minus(drive.getPose().getTranslation());
    // Calculate the angle that delta needs
    final Rotation2d facing = diff.getAngle();

    // Send the request to the drivetrain
    drive.setControl(
        request
            .withVelocityX(linearVelocity.getX())
            .withVelocityY(linearVelocity.getY())
            .withTargetDirection(facing));

    final double dist = diff.getNorm();
    shooter.shootIntoHub(dist);

    if (shooter.atTargetVelocity() && !commandedFeederWheel) {
      indexer.runKickerWheel();
      indexer.runIndexerWheel();

      commandedFeederWheel = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheel();

    indexer.stopIndexerWheel();
    indexer.stopKickerWheel();
  }
}
