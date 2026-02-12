package com.frc6324.robot2026.subsystems.shooter;

import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.HoodConstants.*;
import static edu.wpi.first.units.Units.*;

import com.frc6324.robot2026.sim.MapleSimManager;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

  private Angle hoodSetpoint = Rotations.zero();
  private boolean currentlyShooting = false;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void pass() {
    io.setHoodAngle(HOOD_MAX_ANGLE);
    hoodSetpoint = HOOD_MAX_ANGLE;

    io.setFlywheelVelocity(RPM.of(RobotBase.isSimulation() ? 1200 : 3000));

    currentlyShooting = true;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/CurrentlyShooting", currentlyShooting);
  }

  @Override
  public void simulationPeriodic() {
    final Angle shooterAngle = inputs.hoodPosition;
    final double shooterAngleRads = shooterAngle.in(Radians);

    Rotation3d rot = new Rotation3d(0, shooterAngleRads, 0);
    Translation3d translation = ROBOT_TO_HOOD_AXLE.plus(HOOD_AXLE_TO_HOOD.rotateBy(rot));

    Pose3d hoodPose = new Pose3d(translation, new Rotation3d(0, Math.PI + shooterAngleRads, 0));

    Logger.recordOutput("Shooter/HoodPosition", hoodPose);

    final MapleSimManager manager = MapleSimManager.getInstance();

    manager.setShooterPosition(translation.plus(HOOD_SHOOTING_OFFSET));
    manager.setShooterSpeed(inputs.flywheelLeaderVelocity);
    manager.setShooterAngle(Degrees.of(90).minus(inputs.hoodPosition));

    if (currentlyShooting && shooterAngle.isNear(hoodSetpoint, HOOD_TOLERANCE)) {
      manager.launchFuel();
    }
  }

  public void shootIntoHub(double distanceToHub) {}

  public void spinUpShooter() {
    io.setFlywheelVelocity(RadiansPerSecond.of(75));
    currentlyShooting = false;
  }

  public void stop() {
    io.coastFlywheel();

    currentlyShooting = false;
  }

  public void stowHood() {
    io.setHoodAngle(HOOD_STOW_ANGLE);
    hoodSetpoint = HOOD_STOW_ANGLE;
  }
}
