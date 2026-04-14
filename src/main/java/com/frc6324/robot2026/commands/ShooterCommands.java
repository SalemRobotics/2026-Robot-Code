package com.frc6324.robot2026.commands;

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
import com.frc6324.robot2026.subsystems.leds.LEDs.LEDState;
import com.frc6324.robot2026.subsystems.rollers.Rollers;
import com.frc6324.robot2026.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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
      Rollers rollers,
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
                return new DriverShootIntoHubCommand(
                    drive, indexer, intake, rollers, shooter, controller);
              } else {
                Logger.recordOutput("Commands/ShootCommand/Phase", "Passing");
                return new PassToAllianceZoneCommand(
                    drive, indexer, intake, rollers, shooter, controller);
              }
            },
            Set.of(drive, indexer, intake, shooter))
        .until(switchCondition);
  }

  public static class ShootUpAgainstHubCommand extends Command {
    private final Indexer indexer;
    private final Intake intake;
    private final Rollers rollers;
    private final Shooter shooter;

    private boolean indexerRunning = false;
    private boolean sendingIntakeOut = false;
    private final Timer intakeCommandTimeout = new Timer();

    public ShootUpAgainstHubCommand(
        Indexer indexer, Intake intake, Rollers rollers, Shooter shooter) {
      this.indexer = indexer;
      this.intake = intake;
      this.rollers = rollers;
      this.shooter = shooter;

      addRequirements(indexer, intake, rollers, shooter);
    }

    @Override
    public void end(boolean interrupted) {
      shooter.stopFlywheel();
      rollers.stopRollers();

      indexer.stopKickerWheel();
      indexer.stopIndexerWheel();
      LEDState.closeShooting = false;
    }

    @Override
    public void execute() {
      shooter.shootUpAgainstHub();
      if (!indexerRunning) {
        indexer.runKickerWheel();
        indexer.runIndexerWheel();

        indexerRunning = true;
      }

      final boolean intakeTimedOut = intakeCommandTimeout.hasElapsed(0.420);
      if (sendingIntakeOut) {
        if (intakeTimedOut || intake.isDeployed()) {
          intake.retract();
          sendingIntakeOut = false;
          intakeCommandTimeout.restart();
        }
      } else {
        if (intakeTimedOut || intake.isRetracted()) {
          intake.deploy(true);
          sendingIntakeOut = true;
          intakeCommandTimeout.restart();
        }
      }
    }

    @Override
    public void initialize() {
      indexer.stopIndexerWheel();
      indexer.stopKickerWheel();

      indexerRunning = false;
      sendingIntakeOut = !intake.isDeployed();
      intakeCommandTimeout.start();

      rollers.spinRollers();
      LEDState.closeShooting = true;
    }
  }

  abstract static class AbstractShootAtCommand extends Command {
    protected final SwerveDrive drive;
    protected final Indexer indexer;
    protected final Intake intake;
    protected final Rollers rollers;
    protected final Shooter shooter;
    protected final String logKey;

    protected SwerveRequest.FieldCentricFacingAngle request =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveDrive.DRIVE_REQUEST)
            .withSteerRequestType(SwerveDrive.STEER_REQUEST)
            .withHeadingPID(DriveCommands.POINTING_KP, 0, DriveCommands.POINTING_KD)
            .withDesaturateWheelSpeeds(true);

    private boolean kickerRunning = false;
    private boolean sendingIntakeOut = false;
    private final Timer intakeCommandTimeout = new Timer();

    protected AbstractShootAtCommand(
        SwerveDrive drive,
        Indexer indexer,
        Intake intake,
        Rollers rollers,
        Shooter shooter,
        String name) {
      this.drive = drive;
      this.indexer = indexer;
      this.intake = intake;
      this.rollers = rollers;
      this.shooter = shooter;
      this.logKey = "Commands/" + name;

      setName(name);
      addRequirements(drive, indexer, intake, rollers, shooter);
    }

    protected void applyDriverInput() {}

    abstract void commandShooter(double distanceToTarget);

    @Override
    public void end(boolean interrupted) {
      shooter.stopFlywheel();
      rollers.stopRollers();

      indexer.stopKickerWheel();
      indexer.stopIndexerWheel();
    }

    @Override
    public void execute() {
      // Get the robot and shooter's positions
      final Pose2d robotPose = drive.getPose();

      // Calculate the linear distance to the target
      final Translation2d target = getTarget();
      final Translation2d delta = target.minus(robotPose.getTranslation());

      final Rotation2d facing = delta.getAngle().plus(Rotation2d.k180deg);

      applyDriverInput();
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

        final boolean intakeTimedOut = intakeCommandTimeout.hasElapsed(0.420);
        if (sendingIntakeOut) {
          if (intakeTimedOut || intake.isDeployed()) {
            intake.retract();
            sendingIntakeOut = false;
            intakeCommandTimeout.restart();
          }
        } else {
          if (intakeTimedOut || intake.isRetracted()) {
            intake.deploy(true);
            sendingIntakeOut = true;
            intakeCommandTimeout.restart();
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
      Logger.recordOutput(logKey + "/SendingIntakeOut", sendingIntakeOut);
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
      sendingIntakeOut = !intake.isDeployed();
      intakeCommandTimeout.start();

      rollers.spinRollers();
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

    private final Debouncer debouncer = new Debouncer(0.25);
    private Translation2d hubTranslation;

    public ShootIntoHubCommand(
        SwerveDrive drive,
        Indexer indexer,
        Intake intake,
        Rollers rollers,
        Shooter shooter,
        String name) {
      super(drive, indexer, intake, rollers, shooter, name);
    }

    @Override
    void commandShooter(double distanceToTarget) {
      Logger.recordOutput(logKey + "/DistanceToHub", distanceToTarget);
      shooter.shootIntoHub(distanceToTarget);
    }

    @Override
    public void end(boolean interrupted) {
      LEDState.shooting = false;
      super.end(interrupted);
    }

    @Override
    Translation2d getTarget() {
      return hubTranslation;
    }

    @Override
    public void initialize() {
      hubTranslation = FieldConstants.getAllianceHub().getTranslation();
      super.initialize();

      LEDState.shooting = true;
    }

    @Override
    boolean shouldIndex(Rotation2d targetFacing, double distanceToTarget) {
      final double tolerance = CLOSE_ANGLE_TOLERANCE + TOLERANCE_DECAY_PER_METER * distanceToTarget;

      final Rotation2d robotYaw = drive.getPose().getRotation();
      final boolean atRobotAngle =
          MathUtil.isNear(robotYaw.getRadians(), targetFacing.getRadians(), tolerance);
      final boolean atHoodAngle = shooter.atTargetHoodAngle();

      Logger.recordOutput(logKey + "/AtRobotAngle", atRobotAngle);
      Logger.recordOutput(logKey + "/AtHoodSetpoint", atHoodAngle);

      final boolean should = atRobotAngle && atHoodAngle;
      return debouncer.calculate(should);
    }
  }

  public static class DriverShootIntoHubCommand extends ShootIntoHubCommand {
    private final XboxController controller;

    public DriverShootIntoHubCommand(
        SwerveDrive drive,
        Indexer indexer,
        Intake intake,
        Rollers rollers,
        Shooter shooter,
        CommandXboxController controller) {
      super(drive, indexer, intake, rollers, shooter, "ShootIntoHub");
      this.controller = controller.getHID();
    }

    @Override
    protected void applyDriverInput() {
      Translation2d velocity = DriveCommands.getLinearVelocityFromJoysticks(controller);
      velocity = velocity.times(SwerveDrive.getMaxLinearSpeed() / 3);

      request.VelocityX = velocity.getX();
      request.VelocityY = velocity.getY();
    }
  }

  public static class PassToAllianceZoneCommand extends AbstractShootAtCommand {
    private final XboxController controller;
    private List<Translation2d> allianceZoneTranslations;
    private boolean underTrench;

    public PassToAllianceZoneCommand(
        SwerveDrive drive,
        Indexer indexer,
        Intake intake,
        Rollers rollers,
        Shooter shooter,
        CommandXboxController controller) {
      super(drive, indexer, intake, rollers, shooter, "PassToAllianceZone");
      this.controller = controller.getHID();
    }

    @Override
    protected void applyDriverInput() {
      Translation2d velocity = DriveCommands.getLinearVelocityFromJoysticks(controller);
      velocity = velocity.times(SwerveDrive.getMaxLinearSpeed() / 1.5);

      request.VelocityX = velocity.getX();
      request.VelocityY = velocity.getY();
    }

    @Override
    void commandShooter(double distanceToTarget) {
      if (underTrench) {
        shooter.stowHood();
      } else {
        shooter.pass(distanceToTarget);
      }
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
    public void execute() {
      final Pose2d robotPose = drive.getPose();

      underTrench =
          robotPose.boundedWithinX(LinesVertical.ALLIANCE_ZONE, LinesVertical.NEUTRAL_ZONE_NEAR)
              || robotPose.boundedWithinX(
                  LinesVertical.NEUTRAL_ZONE_FAR, LinesVertical.OPP_ALIANCE_ZONE);

      super.execute();
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

    public IdleShooterCommand(Shooter shooter, PoseSupplier poseSupplier) {
      this.shooter = shooter;
      this.robotPoseSupplier = poseSupplier;

      addRequirements(shooter);
    }

    @Override
    public void initialize() {
      switch (DriverStation.getAlliance().orElse(Alliance.Blue)) {
        case Blue -> {
          allianceZoneStart = 0;
          allianceZoneEnd = LinesVertical.ALLIANCE_ZONE;
        }
        case Red -> {
          allianceZoneStart = LinesVertical.OPP_ALIANCE_ZONE;
          allianceZoneEnd = FieldConstants.FIELD_LENGTH;
        }
      }
    }

    @Override
    public void execute() {
      final Pose2d robotPose = robotPoseSupplier.getPose();

      if (robotPose.boundedWithinX(allianceZoneStart, allianceZoneEnd)) {
        final Pose2d hub = FieldConstants.getAllianceHub();
        final double dist = robotPose.getTranslation().getDistance(hub.getTranslation());

        // Spin up the shooter to a low velocity in the alliance zone to minimize
        // overhead & current
        // use
        shooter.spinUpForHubShot(dist);
      } else {
        shooter.stopFlywheel();
      }

      final Pose2d poseIn1Sec = DrivingUtils.estimateFuturePose(0.5);
      if (robotPose.boundedWithinX(LinesVertical.ALLIANCE_ZONE, LinesVertical.NEUTRAL_ZONE_NEAR)
          || robotPose.boundedWithinX(
              LinesVertical.NEUTRAL_ZONE_FAR, LinesVertical.OPP_ALIANCE_ZONE)
          || poseIn1Sec.boundedWithinX(LinesVertical.ALLIANCE_ZONE, LinesVertical.NEUTRAL_ZONE_NEAR)
          || poseIn1Sec.boundedWithinX(
              LinesVertical.NEUTRAL_ZONE_FAR, LinesVertical.OPP_ALIANCE_ZONE)) {
        // Stow the hood if we're going under the trench
        shooter.stowHood();
      }
    }
  }
}
