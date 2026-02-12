package com.frc6324.robot2026.sim;

import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.LazySingleton;
import com.frc6324.lib.util.UnitsUtils;
import com.frc6324.robot2026.subsystems.drive.DrivetrainConstants;
import com.frc6324.robot2026.subsystems.intake.IntakeConstants;
import com.frc6324.robot2026.subsystems.shooter.ShooterConstants;
import com.frc6324.robot2026.subsystems.shooter.ShooterConstants.FlywheelConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod(UnitsUtils.class)
@LazySingleton
public final class MapleSimManager {
  private static MapleSimManager instance = null;

  private final Arena2026Rebuilt arena;
  private final SwerveDriveSimulation driveSimulation =
      new SwerveDriveSimulation(
          DrivetrainConstants.MAPLE_SIM_CONFIG, DrivetrainConstants.STARTING_POSE);
  private final IntakeSimulation intakeSim =
      IntakeSimulation.OverTheBumperIntake(
          "Fuel",
          driveSimulation,
          Inches.of(27),
          IntakeConstants.INTAKE_EXTENSION,
          IntakeSide.FRONT,
          50);
  private final ReentrantLock simulationLock = new ReentrantLock();

  @Setter private Translation3d shooterPosition = Translation3d.kZero;
  @Setter private AngularVelocity shooterSpeed = RadiansPerSecond.zero();
  @Setter private Angle shooterAngle = Radians.zero();

  private boolean intakeExtended = false;
  private double lastLaunchTimestamp = 0;

  private MapleSimManager() {
    arena = new Arena2026Rebuilt(false);
    arena.setEfficiencyMode(false);
    arena.addDriveTrainSimulation(driveSimulation);
    intakeSim.register(arena);

    SimulatedArena.overrideInstance(arena);

    SmartDashboard.putData(
        "FieldSimulationCommands/DumpBlueOutpost",
        Commands.runOnce(
                () -> {
                  try {
                    simulationLock.lock();
                    arena.outpostDump(true);
                  } finally {
                    simulationLock.unlock();
                  }
                })
            .ignoringDisable(true));
    SmartDashboard.putData(
        "FieldSimulationCommands/DumpRedOutpost",
        Commands.runOnce(
                () -> {
                  try {
                    simulationLock.lock();
                    arena.outpostDump(false);
                  } finally {
                    simulationLock.unlock();
                  }
                })
            .ignoringDisable(true));
    SmartDashboard.putData(
        "FieldSimulationCommands/ResetFieldForAuto",
        Commands.runOnce(
                () -> {
                  try {
                    simulationLock.lock();
                    arena.resetFieldForAuto();
                  } finally {
                    simulationLock.unlock();
                  }
                })
            .ignoringDisable(true));
  }

  public static MapleSimManager getInstance() {
    if (instance == null) {
      instance = new MapleSimManager();
    }

    return instance;
  }

  /**
   * Gets the drivetrain simulation of the <em>main</em> robot; that is, the robot that this
   * application is currently simulating.
   *
   * @return The desired swerve drive simulation.
   */
  public SwerveDriveSimulation getMainRobotDriveSimulation() {
    return driveSimulation;
  }

  public void setIntakeExtended(boolean extended) {
    intakeExtended = extended;

    if (extended) {
      intakeSim.startIntake();
    } else {
      intakeSim.stopIntake();
    }
  }

  public void launchFuel() {
    final double currentTimestamp = Timer.getTimestamp();

    try {
      simulationLock.lock();
      if (currentTimestamp - lastLaunchTimestamp > ShooterConstants.TIME_TO_LAUNCH_FUEL
          && intakeSim.obtainGamePieceFromIntake()) {
        final Pose2d drivePose = driveSimulation.getSimulatedDriveTrainPose();

        final RebuiltFuelOnFly piece =
            new RebuiltFuelOnFly(
                drivePose.getTranslation(),
                shooterPosition.toTranslation2d(),
                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                drivePose.getRotation(),
                Meters.of(shooterPosition.getZ()),
                FlywheelConstants.FLYWHEEL_RADIUS
                    .times(shooterSpeed.in(RadiansPerSecond))
                    .per(Second),
                shooterAngle);

        arena.addGamePieceProjectile(piece);

        lastLaunchTimestamp = currentTimestamp;
      }
    } finally {
      simulationLock.unlock();
    }
  }

  public void simulationPeriodic() {
    arena.simulationPeriodic();

    Logger.recordOutput("FieldSimulation/Fuel", arena.getGamePiecesArrayByType("Fuel"));
    Logger.recordOutput("FieldSimulation/BlueAllianceScore", arena.getScore(Alliance.Blue));
    Logger.recordOutput("FieldSimulation/RedAllianceScore", arena.getScore(Alliance.Red));

    Logger.recordOutput("FieldSimulation/MainRobot/IntakeExtended", intakeExtended);
    Logger.recordOutput("FieldSimulation/MainRobot/IntakeActive", intakeSim.isRunning());
    Logger.recordOutput(
        "FieldSimulation/MainRobot/NumberFuelHeld", intakeSim.getGamePiecesAmount());
  }
}
