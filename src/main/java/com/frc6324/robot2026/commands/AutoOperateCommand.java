package com.frc6324.robot2026.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.frc6324.lib.util.FieldConstants.Hub;
import com.frc6324.lib.util.FieldConstants.LinesVertical;
import com.frc6324.robot2026.Robot;
import com.frc6324.robot2026.subsystems.drive.SwerveDrive;
import com.frc6324.robot2026.subsystems.indexer.Indexer;
import com.frc6324.robot2026.subsystems.intake.Intake;
import com.frc6324.robot2026.subsystems.rollers.Rollers;
import com.frc6324.robot2026.subsystems.shooter.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class AutoOperateCommand extends Command {
  private final SwerveRequest.FieldCentric regularDrive = new SwerveRequest.FieldCentric();
  private final SwerveRequest.FieldCentricFacingAngle hubFacingRequest =
      new SwerveRequest.FieldCentricFacingAngle();
  private final SwerveRequest.ApplyRobotSpeeds autointakeRequest =
      new SwerveRequest.ApplyRobotSpeeds();

  private final CommandXboxController controller;
  private final XboxController rawHID;

  private final SwerveDrive drive;
  private final Indexer indexer;
  private final Intake intake;
  private final Rollers rollers;
  private final Shooter shooter;

  private Alliance currentAlliance = Alliance.Blue;

  public AutoOperateCommand(
      CommandXboxController controller,
      SwerveDrive drive,
      Indexer indexer,
      Intake intake,
      Rollers rollers,
      Shooter shooter) {
    this.controller = controller;
    this.rawHID = controller.getHID();

    this.drive = drive;
    this.indexer = indexer;
    this.intake = intake;
    this.rollers = rollers;
    this.shooter = shooter;

    addRequirements(drive, indexer, intake, rollers, shooter);
  }

  private void executeScoringMode() {
    final Pose2d robotPose = drive.getPose();
    final boolean isBlue = currentAlliance == Alliance.Blue;

    final boolean inAllianceZone;
    final Translation2d allianceHub;

    if (isBlue) {
      inAllianceZone = robotPose.getX() < LinesVertical.ALLIANCE_ZONE;
      allianceHub = Hub.INNER_CENTER_POINT.toTranslation2d();
    } else {
      inAllianceZone = robotPose.getX() > LinesVertical.OPP_ALIANCE_ZONE;
      allianceHub = Hub.OPP_TOP_CENTER_POINT.toTranslation2d();
    }

    if (inAllianceZone) {
      final double distanceToHub = robotPose.getTranslation().getDistance(allianceHub);

      // Automatically start scoring
      shooter.shootIntoHub(distanceToHub);

    } else {
      // Stop the shooter
      shooter.stop();

      // Continue to allow the driver to fully control the robot
      {
        Translation2d linearVelocity = DriveCommands.getLinearVelocityFromJoysticks(rawHID);
      }
    }
  }

  private void executePrepMode() {}

  @Override
  public void initialize() {
    currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  @Override
  public void execute() {
    if (!DriverStation.isEnabled()) {
      return;
    }

    if (Robot.getIsAllianceActive()) {
      executeScoringMode();
    } else {
      executePrepMode();
    }
  }
}
