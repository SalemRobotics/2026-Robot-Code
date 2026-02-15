package com.frc6324.robot2026.sim;

import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.LazySingleton;
import com.frc6324.lib.util.CommonUtils;
import com.frc6324.robot2026.subsystems.drive.DrivetrainConstants;
import com.frc6324.robot2026.subsystems.intake.IntakeConstants;
import com.frc6324.robot2026.subsystems.shooter.ShooterConstants;
import com.frc6324.robot2026.subsystems.shooter.ShooterConstants.FlywheelConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.concurrent.locks.ReentrantLock;
import lombok.experimental.ExtensionMethod;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

/** A helper class to manage maple-sim accross the whole robot in a thread-safe, lazy way. */
@ExtensionMethod(CommonUtils.class)
@LazySingleton
public final class MapleSimManager {
  private static MapleSimManager instance = null;

  private final RebuiltArena arena;
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

  private Translation3d shooterPosition = Translation3d.kZero;
  private AngularVelocity shooterSpeed = RadiansPerSecond.zero();
  private Angle shooterAngle = Radians.zero();

  private boolean intakeExtended = false;
  private double lastLaunchTimestamp = 0;

  private MapleSimManager() {
    arena = new RebuiltArena();

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

  /**
   * Gets an instance of the simulation manager, or initializes it if it hasn't been already.
   *
   * @return The instance of the maple-sim manager.
   */
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

  /**
   * Sets whether the intake is extended (and therefore on).
   *
   * @param extended Whether the intake is extended.
   */
  public void setIntakeExtended(boolean extended) {
    intakeExtended = extended;

    if (extended) {
      intakeSim.startIntake();
    } else if (intakeSim.isRunning()) {
      intakeSim.stopIntake();
    }
  }

  /**
   * Sets the state of the simulated shooter.
   *
   * @param position The position of the center of the shooter.
   * @param speed The speed of the shooter's flywheel.
   * @param angle The angle of the shooter's hood.
   */
  public void setShooterState(Translation3d position, AngularVelocity speed, Angle angle) {
    shooterPosition = position;
    shooterSpeed = speed;
    shooterAngle = angle;
  }

  /** Launches a single fuel given the known shooter state. */
  public void launchFuel() {
    final double currentTimestamp = Timer.getTimestamp();

    try {
      simulationLock.lock();
      if (currentTimestamp - lastLaunchTimestamp > ShooterConstants.TIME_TO_LAUNCH_FUEL
          && intakeSim.obtainGamePieceFromIntake()) {
        final Pose2d drivePose = driveSimulation.getSimulatedDriveTrainPose();

        final LinearVelocity fuelVelocity =
            shooterSpeed
                .getVelocity(FlywheelConstants.FLYWHEEL_RADIUS)
                .times(FlywheelConstants.FLYWHEEL_EFFICIENCY);

        final FlyingFuelSimulation fuel =
            new FlyingFuelSimulation(
                drivePose,
                shooterPosition,
                Rotation2d.kZero,
                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                shooterAngle,
                fuelVelocity);

        arena.addFuelProjectile(fuel);

        lastLaunchTimestamp = currentTimestamp;
      }
    } finally {
      simulationLock.unlock();
    }
  }

  /** Updates maple-sim's simulation and logs helpful data for AdvantageScope. */
  public void simulationPeriodic() {
    try {
      simulationLock.lock();
      arena.simulationPeriodic();

      Logger.recordOutput("FieldSimulation/Fuel", arena.getFuelPoses());
      Logger.recordOutput("FieldSimulation/BlueAllianceScore", arena.getScore(Alliance.Blue));
      Logger.recordOutput("FieldSimulation/RedAllianceScore", arena.getScore(Alliance.Red));

      Logger.recordOutput("FieldSimulation/MainRobot/IntakeExtended", intakeExtended);
      Logger.recordOutput("FieldSimulation/MainRobot/IntakeActive", intakeSim.isRunning());
      Logger.recordOutput(
          "FieldSimulation/MainRobot/NumberFuelHeld", intakeSim.getGamePiecesAmount());
    } finally {
      simulationLock.unlock();
    }
  }
}
