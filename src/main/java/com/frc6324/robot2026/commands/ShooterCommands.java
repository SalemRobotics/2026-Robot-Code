package com.frc6324.robot2026.commands;

import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.SHOOTER_POSITION;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.frc6324.lib.UninstantiableClass;
import com.frc6324.lib.util.AllianceFlipUtil;
import com.frc6324.lib.util.Allocated;
import com.frc6324.lib.util.FieldConstants;
import com.frc6324.lib.util.FieldConstants.LinesVertical;
import com.frc6324.lib.util.PoseExtensions;
import com.frc6324.lib.util.PoseExtensions.PoseSupplier;
import com.frc6324.robot2026.sim.MapleSimManager;
import com.frc6324.robot2026.subsystems.drive.DrivingUtils;
import com.frc6324.robot2026.subsystems.drive.SwerveDrive;
import com.frc6324.robot2026.subsystems.indexer.Indexer;
import com.frc6324.robot2026.subsystems.intake.Intake;
import com.frc6324.robot2026.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod(PoseExtensions.class)
@UninstantiableClass
public final class ShooterCommands {
  private ShooterCommands() {
    throw new IllegalAccessError();
  }

  /**
   * Creates the command that manages
   *
   * @param drive
   * @param indexer
   * @param intake
   * @param shooter
   * @param controller
   * @return
   */
  public static Command genericShootCommand(
      SwerveDrive drive,
      Indexer indexer,
      Intake intake,
      Shooter shooter,
      CommandXboxController controller) {
    final Allocated<Boolean> inAllianceZone = new Allocated<>(false);

    final BooleanSupplier switchCondition =
        () -> {
          final boolean inAlliance = FieldConstants.isInAllianceZone(drive.getPose());

          if (inAllianceZone.get() != inAlliance) {
            inAllianceZone.set(inAlliance);
            return true;
          }

          return false;
        };

    return Commands.defer(
            () -> {
              final boolean inAlliance = FieldConstants.isInAllianceZone(drive.getPose());

              inAllianceZone.set(inAlliance);
              if (inAlliance) {
                Logger.recordOutput("Commands/ShootCommand/Phase", "Hub");
                return new ShootIntoHubCommand(drive, indexer, intake, shooter, controller);
              } else {
                Logger.recordOutput("Commands/ShootCommand/Phase", "Passing");
                return new PassToAllianceZoneCommand(drive, indexer, intake, shooter, controller);
              }
            },
            Set.of(drive, indexer, intake, shooter))
        .until(switchCondition);
  }

  abstract static class AbstractShootAtCommand extends Command {
    protected final XboxController controller;
    protected final SwerveDrive drive;
    protected final Indexer indexer;
    protected final Intake intake;
    protected final Shooter shooter;
    protected final String logKey;
    private final double driveSpeedReduction;
    private final SwerveRequest.FieldCentricFacingAngle request =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveDrive.DRIVE_REQUEST)
            .withSteerRequestType(SwerveDrive.STEER_REQUEST)
            .withHeadingPID(DriveCommands.POINTING_KP, 0, DriveCommands.POINTING_KD)
            .withDesaturateWheelSpeeds(true);
    private boolean kickerRunning = false;
    private boolean sendIntakeOut = false;

    protected AbstractShootAtCommand(
        SwerveDrive drive,
        Indexer indexer,
        Intake intake,
        Shooter shooter,
        CommandXboxController controller,
        String name,
        double driveSpeedReduction) {
      this.drive = drive;
      this.indexer = indexer;
      this.intake = intake;
      this.shooter = shooter;
      this.controller = controller.getHID();
      this.logKey = "Commands/" + name;
      this.driveSpeedReduction = driveSpeedReduction;

      addRequirements(drive, shooter, indexer);
    }

    abstract void commandShooter(double distanceToTarget);

    @Override
    public void end(boolean interrupted) {
      shooter.stopFlywheel();

      indexer.stopKickerWheel();
      indexer.stopIndexerWheel();
    }

    @Override
    public final void execute() {
      // Get the linear velocity for the drivetrain
      Translation2d linearVelocity = DriveCommands.getLinearVelocityFromJoysticks(controller);
      linearVelocity = linearVelocity.times(SwerveDrive.getMaxLinearSpeed() / driveSpeedReduction);

      // Apply the velocities to the swerve request
      request.VelocityX = linearVelocity.getX();
      request.VelocityY = linearVelocity.getY();

      // Get the robot and shooter's positions
      final Pose2d robotPose = drive.getPose();
      final Pose2d shooterPosition = robotPose.transformBy(SHOOTER_POSITION);

      // Calculate the linear distance to the target
      final Translation2d target = getTarget();
      final Translation2d delta = target.minus(shooterPosition.getTranslation());
      final Rotation2d facing = delta.getAngle();

      drive.setControl(request.withTargetDirection(AllianceFlipUtil.apply(facing)));

      // Command the shooter
      final double distance = delta.getNorm();
      commandShooter(distance);

      // Conditionally start/stop indexing
      final boolean index = shouldIndex(facing, distance);
      if (index) {
        indexer.runIndexerWheel();

        if (!kickerRunning) {
          indexer.runKickerWheel();
          kickerRunning = true;
        }

        if (sendIntakeOut) {
          if (intake.isDeployed()) {
            sendIntakeOut = false;
          } else {
            // Send the intake out
            intake.deploy();
          }
        } else {
          if (intake.isRetracted()) {
            sendIntakeOut = true;
          } else {
            // Send the intake in
            intake.retract();
          }
        }
      } else {
        if (kickerRunning) {
          indexer.stopIndexerWheel();
          indexer.stopKickerWheel();
        }

        kickerRunning = false;
      }

      // Launch a fuel during simulation
      if (RobotBase.isSimulation()) {
        MapleSimManager.getInstance().launchFuel();
      }

      // Log data about the command to preserve sanity
      Logger.recordOutput(logKey + "/TargetPose", target);
      Logger.recordOutput(logKey + "/TargetHeading", facing);
      Logger.recordOutput(logKey + "/DistanceToTarget", distance);
      Logger.recordOutput(logKey + "/Indexing", index);
      Logger.recordOutput(logKey + "/SendingIntakeOut", sendIntakeOut);
    }

    /**
     * Gets the target of fuel launched by this command.
     *
     * @return The translation of the target (e.g. alliance zone, the hub).
     */
    abstract Translation2d getTarget();

    @Override
    public void initialize() {
      indexer.stopIndexerWheel();
      indexer.stopKickerWheel();

      kickerRunning = false;
      sendIntakeOut = !intake.isDeployed();
    }

    /**
     * Determines whether the robot's current state is good enough to start actively launching fuel.
     *
     * @param targetFacing The targeted facing of the robot.
     * @param distanceToTarget The distance from the shooter to the target.
     * @return Whether the indexer should be running.
     */
    abstract boolean shouldIndex(Rotation2d targetFacing, double distanceToTarget);
  }

  public static class ShootIntoHubCommand extends AbstractShootAtCommand {
    private static final double CLOSE_ANGLE_TOLERANCE = Units.degreesToRadians(7.5);
    private static final double TOLERANCE_DECAY_PER_METER = Units.degreesToRadians(-1);
    private Translation2d hubTranslation;

    public ShootIntoHubCommand(
        SwerveDrive drive,
        Indexer indexer,
        Intake intake,
        Shooter shooter,
        CommandXboxController controller) {
      super(drive, indexer, intake, shooter, controller, "ShootIntoHub", 3);
    }

    @Override
    void commandShooter(double distanceToTarget) {
      Logger.recordOutput(logKey + "/DistanceToHub", distanceToTarget);
      shooter.shootIntoHub(distanceToTarget);
    }

    @Override
    Translation2d getTarget() {
      return hubTranslation;
    }

    @Override
    public void initialize() {
      hubTranslation = FieldConstants.getAllianceHub().getTranslation();
      super.initialize();
    }

    @Override
    boolean shouldIndex(Rotation2d targetFacing, double distanceToTarget) {
      final double tolerance = CLOSE_ANGLE_TOLERANCE + TOLERANCE_DECAY_PER_METER * distanceToTarget;

      final Rotation2d robotYaw = drive.getPose().getRotation();
      final boolean atRobotAngle = MathUtil.isNear(robotYaw.getRadians(), targetFacing.getRadians(), tolerance);
      final boolean atHoodAngle = shooter.atTargetHoodAngle();
      final boolean atFlyVelocity = shooter.atTargetVelocity();

      Logger.recordOutput(logKey + "/RobotAtAngle", atRobotAngle);
      Logger.recordOutput(logKey + "/AtHoodSetpoint", atHoodAngle);
      Logger.recordOutput(logKey + "/AtTargetVelocity", atFlyVelocity);
      return atRobotAngle && atHoodAngle && atFlyVelocity;
    }
  }

  public static class PassToAllianceZoneCommand extends AbstractShootAtCommand {
    private List<Translation2d> allianceZoneTranslations;

    public PassToAllianceZoneCommand(
        SwerveDrive drive,
        Indexer indexer,
        Intake intake,
        Shooter shooter,
        CommandXboxController controller) {
      super(drive, indexer, intake, shooter, controller, "PassToAllianceZone", 1.5);
    }

    @Override
    void commandShooter(double distanceToTarget) {
      shooter.pass(distanceToTarget);
    }

    @Override
    Translation2d getTarget() {
      return drive.getPose().getTranslation().nearest(allianceZoneTranslations);
    }

    @Override
    public void initialize() {
      final double x =
          switch (DriverStation.getAlliance().orElse(Alliance.Blue)) {
            case Blue -> LinesVertical.ALLIANCE_ZONE / 2;
            case Red -> (LinesVertical.OPP_ALIANCE_ZONE + FieldConstants.FIELD_LENGTH) / 2;
          };

      allianceZoneTranslations =
          List.of(
              new Translation2d(x, FieldConstants.FIELD_WIDTH / 5),
              new Translation2d(x, FieldConstants.FIELD_WIDTH / 4),
              new Translation2d(x, FieldConstants.FIELD_WIDTH * 0.75),
              new Translation2d(x, FieldConstants.FIELD_WIDTH * 0.8));

      Logger.recordOutput(
          logKey + "/AllPassingPoses", allianceZoneTranslations.toArray(Translation2d[]::new));
      super.initialize();
    }

    @Override
    boolean shouldIndex(Rotation2d targetFacing, double distanceToTarget) {
      return true;
    }
  }

  public static class IdleShooterCommand extends Command {
    private final Shooter shooter;
    private final PoseSupplier robotPoseSupplier;

    private double allianceZoneStart;
    private double allianceZoneEnd;

    private double trenchStart;
    private double trenchEnd;

    public IdleShooterCommand(Shooter shooter, SwerveDrive drive) {
      this.shooter = shooter;
      this.robotPoseSupplier = drive;

      addRequirements(shooter);
    }

    @Override
    public void initialize() {
      switch (DriverStation.getAlliance().orElse(Alliance.Blue)) {
        case Blue -> {
          allianceZoneStart = 0;
          allianceZoneEnd = LinesVertical.ALLIANCE_ZONE;

          trenchStart = LinesVertical.ALLIANCE_ZONE;
          trenchEnd = LinesVertical.NEUTRAL_ZONE_NEAR;
        }
        case Red -> {
          allianceZoneStart = LinesVertical.OPP_ALIANCE_ZONE;
          allianceZoneEnd = FieldConstants.FIELD_LENGTH;

          trenchStart = LinesVertical.NEUTRAL_ZONE_FAR;
          trenchEnd = LinesVertical.OPP_ALIANCE_ZONE;
        }
      }
    }

    @Override
    public void execute() {
      final Pose2d robotPose = robotPoseSupplier.getPose();

      if (robotPose.boundedWithinX(allianceZoneStart, allianceZoneEnd)) {
        final Pose2d hub = FieldConstants.getAllianceHub();
        final double dist = robotPose.getTranslation().getDistance(hub.getTranslation());

        // Spin up the shooter to a low velocity in the alliance zone to minimize overhead & current
        // use
        shooter.spinUpForHubShot(dist);
      } else {
        shooter.stopFlywheel();
      }

      final Pose2d poseIn1Sec = DrivingUtils.estimateFuturePose(0.5);
      if (robotPose.boundedWithinX(trenchStart, trenchEnd)
          || poseIn1Sec.boundedWithinX(trenchStart, trenchEnd)) {
        // Stow the hood if we're going under the trench
        shooter.stowHood();
      }
    }
  }
}
