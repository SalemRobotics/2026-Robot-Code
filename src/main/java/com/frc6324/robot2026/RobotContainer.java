/*
 * Copyright (c) 2025 The Blue Devils.
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */
package com.frc6324.robot2026;

import static com.frc6324.robot2026.Constants.*;
import static com.frc6324.robot2026.generated.TunerConstants.BackLeft;
import static com.frc6324.robot2026.generated.TunerConstants.BackRight;
import static com.frc6324.robot2026.generated.TunerConstants.FrontLeft;
import static com.frc6324.robot2026.generated.TunerConstants.FrontRight;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.frc6324.lib.util.AllianceFlipUtil;
import com.frc6324.lib.util.FieldConstants.LinesHorizontal;
import com.frc6324.lib.util.FieldConstants.LinesVertical;
import com.frc6324.lib.util.PoseExtensions;
import com.frc6324.lib.util.logging.IOLayer;
import com.frc6324.lib.util.logging.LoggedTracer;
import com.frc6324.robot2026.commands.DriveCommands;
import com.frc6324.robot2026.commands.ShooterCommands;
import com.frc6324.robot2026.commands.ShooterCommands.*;
import com.frc6324.robot2026.commands.autos.Auto;
import com.frc6324.robot2026.commands.autos.NeutralZoneAutos;
import com.frc6324.robot2026.sim.MapleSimManager;
import com.frc6324.robot2026.subsystems.drive.Drive;
import com.frc6324.robot2026.subsystems.drive.can.CANBusIOCANivore;
import com.frc6324.robot2026.subsystems.drive.gyro.GyroIOPigeon2;
import com.frc6324.robot2026.subsystems.drive.gyro.GyroIOSim;
import com.frc6324.robot2026.subsystems.drive.module.ModuleIOReplay;
import com.frc6324.robot2026.subsystems.drive.module.ModuleIOSim;
import com.frc6324.robot2026.subsystems.drive.module.ModuleIOTalonFX;
import com.frc6324.robot2026.subsystems.drive.odometry.OdometryThreadReal;
import com.frc6324.robot2026.subsystems.drive.odometry.OdometryThreadSim;
import com.frc6324.robot2026.subsystems.indexer.*;
import com.frc6324.robot2026.subsystems.intake.*;
import com.frc6324.robot2026.subsystems.leds.LEDs;
import com.frc6324.robot2026.subsystems.leds.LEDs.LEDState;
import com.frc6324.robot2026.subsystems.rollers.*;
import com.frc6324.robot2026.subsystems.shooter.*;
import com.frc6324.robot2026.subsystems.vision.apriltag.*;
import com.frc6324.robot2026.subsystems.vision.objdetect.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Arrays;
import lombok.experimental.ExtensionMethod;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@ExtensionMethod(PoseExtensions.class)
@SuppressWarnings("unused")
public class RobotContainer {
  private final AprilTagVision apriltag;
  private final Indexer indexer;
  private final Intake intake;
  private final LEDs leds = new LEDs();
  private final Rollers rollers;
  private final Shooter shooter;
  private final Drive drive;

  private final PowerDistribution pdh = new PowerDistribution();
  private final LoggedPowerDistribution loggedPDH =
      LoggedPowerDistribution.getInstance(pdh.getModule(), pdh.getType());

  private final LoggedDashboardChooser<Auto> autoChooser =
      new LoggedDashboardChooser<>("Auto Selection");
  private final CommandXboxController controller =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  public final Field2d field = new Field2d();
  private Command cachedAutoCommand = null;
  private Pose2d cachedAutoStartingPose = null;
  private Pose2d[] rawAutoPreviewPoses = new Pose2d[] {};

  public RobotContainer() {
    pdh.setSwitchableChannel(true);

    switch (Constants.CURRENT_MODE) {
      case REAL -> {
        LoggedTracer.reset();

        final OdometryThreadReal odometryThread = new OdometryThreadReal();
        drive =
            new Drive(
                new ModuleIOTalonFX(odometryThread, FrontLeft),
                new ModuleIOTalonFX(odometryThread, FrontRight),
                new ModuleIOTalonFX(odometryThread, BackLeft),
                new ModuleIOTalonFX(odometryThread, BackRight),
                new CANBusIOCANivore(),
                new GyroIOPigeon2(odometryThread),
                odometryThread);

        LoggedTracer.record("Init/Drive init");

        intake = new Intake(new IntakeIOTalonFX());
        LoggedTracer.record("Init/Intake init");
        apriltag =
            new AprilTagVision(
                new AprilTagIOPhoton(), new AprilTagIOPhoton(), new AprilTagIOPhoton());
        LoggedTracer.record("Init/AprilTag init");

        indexer = new Indexer(new IndexerIOTalonFX());
        LoggedTracer.record("Init/Indexer init");

        rollers = new Rollers(new RollerIOTalonFX());
        LoggedTracer.record("Init/Rollers init");

        shooter = new Shooter(new ShooterIOTalonFX());
        LoggedTracer.record("Init/Shooter init");
      }
      case SIM -> {
        LoggedTracer.reset();

        final SwerveDriveSimulation simulation =
            MapleSimManager.getInstance().getMainRobotDriveSimulation();
        final SwerveModuleSimulation[] modules = simulation.getModules();

        drive =
            new Drive(
                new ModuleIOSim(modules[0], FrontRight),
                new ModuleIOSim(modules[1], FrontRight),
                new ModuleIOSim(modules[2], FrontRight),
                new ModuleIOSim(modules[3], FrontRight),
                IOLayer::replay,
                new GyroIOSim(simulation.getGyroSimulation()),
                new OdometryThreadSim());

        LoggedTracer.record("Init/Drive init");

        apriltag =
            new AprilTagVision(new AprilTagIOSim(), new AprilTagIOSim(), new AprilTagIOSim());
        LoggedTracer.record("Init/AprilTag init");

        indexer = new Indexer(new IndexerIOSim());
        LoggedTracer.record("Init/Indexer init");

        intake = new Intake(new IntakeIOSim());
        LoggedTracer.record("Init/Intake init");

        rollers = new Rollers(new RollerIOSim());
        LoggedTracer.record("Init/Rollers init");

        shooter = new Shooter(new ShooterIOSim());
        LoggedTracer.record("Init/Shooter init");
      }
      default -> {
        LoggedTracer.reset();
        drive =
            new Drive(
                new ModuleIOReplay(),
                new ModuleIOReplay(),
                new ModuleIOReplay(),
                new ModuleIOReplay(),
                IOLayer::replay,
                IOLayer::replay,
                IOLayer::replay);

        LoggedTracer.record("Init/Drive init");

        apriltag = new AprilTagVision(IOLayer::replay, IOLayer::replay, IOLayer::replay);
        LoggedTracer.record("Init/AprilTag init");

        indexer = new Indexer(IOLayer::replay);
        LoggedTracer.record("Init/Indexer init");

        intake = new Intake(IOLayer::replay);
        LoggedTracer.record("Init/Intake init");

        rollers = new Rollers(IOLayer::replay);
        LoggedTracer.record("Init/Rollers init");

        shooter = new Shooter(IOLayer::replay);
        LoggedTracer.record("Init/Shooter init");
      }
    }

    configureBindings();
    LoggedTracer.record("Init/Controller Bindings");

    autoChooser.addDefaultOption("No Auto", Auto.doNothing());

    final Auto.Builder builder =
        new Auto.Builder(RobotState.getInstance(), drive, indexer, intake, rollers, shooter);
    NeutralZoneAutos.addToChooser(autoChooser, builder);

    autoChooser.onChange(
        auto -> {
          if (auto == null) {
            rawAutoPreviewPoses = new Pose2d[0];
            field.getObject("Auto Start").setPoses(rawAutoPreviewPoses);
            field.getObject("Auto Path").setPoses(rawAutoPreviewPoses);

            cachedAutoCommand = null;
            cachedAutoStartingPose = null;

            return;
          }

          final Pose2d[] pathPoses = auto.previewPoses().toArray(Pose2d[]::new);
          if (pathPoses.length == 0) {
            rawAutoPreviewPoses = new Pose2d[0];
            field.getObject("Auto Start").setPoses(rawAutoPreviewPoses);
            field.getObject("Auto Path").setPoses(rawAutoPreviewPoses);
            return;
          }

          pathPoses[0] = auto.startingPose();
          rawAutoPreviewPoses = pathPoses;

          // Apply alliance flip for initial preview
          final Pose2d[] flippedPoses =
              Arrays.stream(rawAutoPreviewPoses)
                  .map(AllianceFlipUtil::apply)
                  .toArray(Pose2d[]::new);

          field.getObject("Auto Start").setPose(flippedPoses[0]);
          field.getObject("Auto Path").setPoses(flippedPoses);

          Logger.recordOutput("Auto/PathPoses", flippedPoses);

          cachedAutoCommand = auto.command();
          cachedAutoStartingPose = auto.startingPose();
        });

    SmartDashboard.putData("Auto Preview", field);
    LoggedTracer.record("Init/Auto chooser");
  }

  private void configureBindings() {
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, controller.getHID()));
    shooter.setDefaultCommand(new IdleShooterCommand(shooter, RobotState.getInstance()::getPose));

    intake.setDefaultCommand(
        intake.run(
            () -> {
              if (RobotState.getInstance()
                  .getPose()
                  .boundedWithinX(
                      LinesVertical.NEUTRAL_ZONE_NEAR, LinesVertical.NEUTRAL_ZONE_FAR)) {
                intake.deploy(true);
              }
            }));
    rollers.setDefaultCommand(
        rollers.runEnd(
            () -> {
              final Pose2d drivePose = RobotState.getInstance().getPose();
              if (drivePose.boundedWithinX(
                  LinesVertical.NEUTRAL_ZONE_NEAR, LinesVertical.NEUTRAL_ZONE_FAR)) {
                rollers.spinRollers();
                LEDState.intaking = true;
              } else {
                rollers.stopRollers();
                LEDState.intaking = false;
              }
            },
            () -> {
              rollers.stopRollers();
              LEDState.intaking = false;
            }));

    indexer.setDefaultCommand(
        indexer.run(
            () -> {
              indexer.stopIndexerWheel();
              indexer.stopKickerWheel();
            }));

    controller
        .leftTrigger()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  intake.deploy(true);
                  rollers.spinRollers();

                  LEDState.intaking = true;
                },
                () -> LEDState.intaking = false,
                intake,
                rollers))
        .onFalse(rollers.run(rollers::stopRollers));

    controller.a().whileTrue(intake.run(intake::retract));
    controller.b().whileTrue(new ShootUpAgainstHubCommand(indexer, intake, rollers, shooter));

    controller
        .rightBumper()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  intake.deploy(true);
                  rollers.outtake();

                  LEDState.outtaking = true;
                },
                () -> LEDState.outtaking = false,
                rollers,
                intake));

    controller
        .rightTrigger()
        .whileTrue(
            ShooterCommands.genericShootCommand(
                drive, indexer, intake, rollers, shooter, controller));

    controller.povUp().onTrue(Commands.runOnce(shooter::incrementOffset));
    controller.povDown().onTrue(Commands.runOnce(shooter::decrementOffset));
    controller.back().onTrue(Commands.runOnce(shooter::resetOffset));
  }

  public Command getAutonomousCommand() {
    if (cachedAutoCommand != null) {
      final Command cmd = cachedAutoCommand;

      // Reset auto cache
      cachedAutoCommand = null;
      cachedAutoStartingPose = null;

      return cmd;
    }

    final Auto option = autoChooser.get();
    return option == null ? Commands.none() : option.command();
  }

  public Pose2d getAutonomousStartingPose() {
    return cachedAutoStartingPose;
  }

  /**
   * Updates the checks displayed on the driver station. These include:
   *
   * <ul>
   *   <li>Whether the robot is on the wrong side of the alliance
   *   <li>Whether the robot is near its starting point rotationally and transationally
   *   <li>Whether the odometry thinks that the robot is on the correct alliance
   * </ul>
   */
  public void updateAutoChecks() {
    final Pose2d drivePose = RobotState.getInstance().getPose();

    final double robotX = drivePose.getX();
    final double robotY = drivePose.getY();

    final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    final boolean onCorrectAlliance =
        (drivePose.getX() > LinesVertical.CENTER && alliance == Alliance.Red)
            || (drivePose.getX() < LinesVertical.CENTER && alliance == Alliance.Blue);
    Logger.recordOutput("AutoChecks/OdometryOnCorrectAlliance", onCorrectAlliance);

    if (cachedAutoStartingPose == null) {
      // No auto selected, return
      return;
    }

    final double startX = cachedAutoStartingPose.getX();
    final double startY = cachedAutoStartingPose.getY();

    final boolean onCorrectSide =
        (startX < LinesHorizontal.CENTER - 0.1 && robotX < LinesHorizontal.CENTER - 0.1)
            || (startX > LinesHorizontal.CENTER + 0.1 && robotX > LinesHorizontal.CENTER + 0.1);
    Logger.recordOutput("AutoChecks/RobotOnCorrectSide", onCorrectSide);

    final boolean withinXTolerance = MathUtil.isNear(robotX, startX, 0.05);
    final boolean withinYTolerance = MathUtil.isNear(robotY, startY, 0.05);

    Logger.recordOutput("AutoChecks/WithinXTolerance", withinXTolerance);
    Logger.recordOutput("AutoChecks/WithinYTolerance", withinYTolerance);

    final boolean withinHeadingTolerance =
        MathUtil.isNear(
            drivePose.getRotation().getRadians(),
            cachedAutoStartingPose.getRotation().getRadians(),
            Radians.convertFrom(10, Degrees));
    Logger.recordOutput("AutoChecks/WithinHeadingTolerance", withinHeadingTolerance);
  }

  /** Updates the robot's pose on the field widget sent to the driver station. */
  public void updateFieldWidget() {
    field.setRobotPose(RobotState.getInstance().getPose());
  }
}
