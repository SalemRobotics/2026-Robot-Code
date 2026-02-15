package com.frc6324.robot2026.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.frc6324.lib.util.FieldConstants;
import com.frc6324.lib.util.PoseExtensions;
import com.frc6324.lib.util.FieldConstants.Hub;
import com.frc6324.lib.util.FieldConstants.LinesVertical;
import com.frc6324.robot2026.Robot;
import com.frc6324.robot2026.subsystems.drive.SwerveDrive;
import com.frc6324.robot2026.subsystems.indexer.Indexer;
import com.frc6324.robot2026.subsystems.intake.Intake;
import com.frc6324.robot2026.subsystems.rollers.Rollers;
import com.frc6324.robot2026.subsystems.shooter.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(PoseExtensions.class)
public final class AutoOperateCommand extends Command {
  private static final double TRIGGER_THRESHOLD = 0.5;
 
  private static final double HUB_PID_KP = 2;
  private static final double HUB_PID_KI = 0;
  private static final double HUB_PID_KD = 0.01;
  private static final double HUB_ROTATION_TOLERANCE = Units.degreesToRadians(5);

  private double allianceStart = 0;
  private double allianceEnd = 0; 
  
  private final SwerveRequest.FieldCentric regularDrive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(SwerveDrive.DRIVE_REQUEST)
      .withSteerRequestType(SwerveDrive.STEER_REQUEST)
      .withDesaturateWheelSpeeds(true);
  
  private final SwerveRequest.FieldCentricFacingAngle hubFacingRequest = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(SwerveDrive.DRIVE_REQUEST)
      .withSteerRequestType(SwerveDrive.STEER_REQUEST)
      .withDesaturateWheelSpeeds(true)
      .withHeadingPID(HUB_PID_KP, HUB_PID_KI, HUB_PID_KD);

  private final SwerveRequest.ApplyRobotSpeeds autointakeRequest =
      new SwerveRequest.ApplyRobotSpeeds();

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
    this.rawHID = controller.getHID();

    this.drive = drive;
    this.indexer = indexer;
    this.intake = intake;
    this.rollers = rollers;
    this.shooter = shooter;

    addRequirements(drive, indexer, intake, rollers, shooter);
  }  
  
  @Override
  public void initialize() {
    currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    allianceStart = switch (currentAlliance) {
      case Blue -> 0;
      case Red -> LinesVertical.OPP_ALIANCE_ZONE;
    };
    allianceEnd = switch (currentAlliance) {
      case Blue -> LinesVertical.ALLIANCE_ZONE;
      case Red -> FieldConstants.FIELD_LENGTH;
    };
  }

  @Override
  public void execute() {
    if (!DriverStation.isEnabled()) {
      return;
    }

    final Pose2d robotPose = drive.getPose();
    if (Robot.getIsAllianceActive() && robotPose.boundedWithinX(allianceStart, allianceEnd)) {
      executeScoringMode();
    } else {
      executeInactiveMode();
    }
  }

  private void driverControl() {
    final Translation2d translation = DriveCommands.getLinearVelocityFromJoysticks(rawHID);

    drive.setControl(regularDrive
      .withVelocityX(translation.getX())
      .withVelocityY(translation.getY()));
  }

  private void executeScoringMode() {    
    if (rawHID.getRightTriggerAxis() > TRIGGER_THRESHOLD) {
      final Pose2d robotPose = drive.getPose();

      final Translation2d robotToHub = Hub.TOP_CENTER_POINT.toTranslation2d().minus(robotPose.getTranslation());
      final Rotation2d facing = robotToHub.getAngle();
      
      final Translation2d translation = DriveCommands.getLinearVelocityFromJoysticks(rawHID)
        .times(SwerveDrive.getMaxLinearSpeed());
      
      drive.setControl(
        hubFacingRequest.withTargetDirection(facing)
          .withVelocityX(translation.getX())
          .withVelocityY(translation.getY()));
      
      if (MathUtil.isNear(facing.getRadians(), robotPose.getRotation().getRadians(), HUB_ROTATION_TOLERANCE)) {
        // command the shooter to shoot and indexer to run
      } else {
        // ensure the indexer is stopped, but start to spin up the shooter
      }
    } else {
      driverControl();
    }
  }

  private void executeInactiveMode() {
    driverControl();

    final Pose2d drivePose = drive.getPose();
    if (drivePose.boundedWithinX(LinesVertical.NEUTRAL_ZONE_NEAR, LinesVertical.NEUTRAL_ZONE_FAR)) {
      // spin up the intake's rollers
    }
  }
}
