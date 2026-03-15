package com.frc6324.robot2026.commands;

import com.frc6324.lib.UninstantiableClass;
import com.frc6324.lib.util.AllianceFlipUtil;
import com.frc6324.lib.util.FieldConstants.LinesHorizontal;
import com.frc6324.lib.util.FieldConstants.LinesVertical;
import com.frc6324.lib.util.PoseExtensions;
import com.frc6324.robot2026.commands.ShooterCommands.ShootIntoHubCommand;
import com.frc6324.robot2026.subsystems.drive.SwerveDrive;
import com.frc6324.robot2026.subsystems.indexer.Indexer;
import com.frc6324.robot2026.subsystems.intake.Intake;
import com.frc6324.robot2026.subsystems.rollers.Rollers;
import com.frc6324.robot2026.subsystems.shooter.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod(PoseExtensions.class)
@UninstantiableClass
public final class AutoCommands {
  public static enum AllianceSide {
    Left("left", LinesHorizontal.LEFT_TRENCH_OPEN_START, LinesHorizontal.LEFT_TRENCH_OPEN_END),
    Right("right", LinesHorizontal.RIGHT_TRENCH_OPEN_START, LinesHorizontal.RIGHT_TRENCH_OPEN_END);

    public final String sideName;
    public final double trenchStart;
    public final double trenchEnd;
    public final double trenchMiddle;
    public final double sidewaysY;

    private AllianceSide(String name, double trenchStart, double trenchEnd) {
      this.sideName = name;
      this.trenchStart = trenchStart;
      this.trenchMiddle = (trenchStart + trenchEnd) / 2;
      this.sidewaysY = trenchStart + (trenchEnd - trenchStart) / 3;
      this.trenchEnd = trenchEnd;
    }

    public Translation2d apply(double x, double y) {
      if (this == Right) {
        y = AllianceFlipUtil.applyYUnchecked(y);
      }

      return new Translation2d(x, y);
    }

    public Translation2d apply(Translation2d translation) {
      return apply(translation.getX(), translation.getY());
    }

    /**
     * Flips a pose on the current alliance from the <em>left</em> to the right side.
     *
     * @param pose
     * @return
     */
    public Pose2d apply(Pose2d pose) {
      return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    }

    public Rotation2d apply(Rotation2d rot) {
      return this == Right ? rot.plus(Rotation2d.k180deg) : rot;
    }
  }

  private AutoCommands() {
    throw new IllegalAccessError();
  }

  public static Command trenchAuto(
      SwerveDrive drive,
      Indexer indexer,
      Intake intake,
      Rollers rollers,
      Shooter shooter,
      AllianceSide side) {
    final FollowPath.Builder builder = drive.getBLineBuilder();

    final Translation2d nearTrenchTranslation =
        new Translation2d(LinesVertical.ALLIANCE_ZONE, side.trenchMiddle);
    final Translation2d farTrenchTranslation =
        new Translation2d(LinesVertical.NEUTRAL_ZONE_NEAR, side.trenchMiddle);

    final Pose2d startPose = new Pose2d(nearTrenchTranslation, Rotation2d.k180deg);
    final Pose2d firstIntakePose = side.apply(new Pose2d(7.8, 7, Rotation2d.fromDegrees(270)));

    return Commands.sequence(
        Commands.race(
            DriveCommands.driveToPose(drive, startPose),
            intake.run(intake::deploy),
            Commands.waitUntil(intake::isSafeToTrench)),
        Commands.waitSeconds(0.2),
        builder.build(
            new Path(
                new Path.Waypoint(startPose),
                new Path.TranslationTarget(farTrenchTranslation),
                new Path.Waypoint(firstIntakePose),
                new Path.TranslationTarget(side.apply(firstIntakePose.getX(), 4.4)),
                new Path.Waypoint(6, side.trenchMiddle, Rotation2d.k180deg),
                new Path.TranslationTarget(nearTrenchTranslation.getX() - 0.5, side.trenchMiddle))),
        new ShootIntoHubCommand(drive, indexer, intake, rollers, shooter, "AutoShootIntoHub"));
  }
}
