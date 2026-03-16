package com.frc6324.robot2026.subsystems.shooter;

import static com.frc6324.lib.util.CommonUtils.NINETY_DEGREES;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.*;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.FlywheelConstants.*;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.HoodConstants.*;
import static edu.wpi.first.units.Units.*;

import com.frc6324.lib.util.LoggedTracer;
import com.frc6324.robot2026.sim.MapleSimManager;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

  private Angle hoodSetpoint = Rotations.zero();
  private boolean hoodAtSetpoint = false;
  private AngularVelocity flywheelSetpoint = RadiansPerSecond.zero();
  private boolean flywheelAtSetpoint = false;

  /**
   * Creates a new shooter subsystem.
   *
   * @param io The implementation of the shooter's I/O to use.
   */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public boolean atTargetHoodAngle() {
    return hoodAtSetpoint;
  }

  /**
   * Gets whether the flywheel has reached its target velocity.
   *
   * @return Whether the flywheel has gotten within 1 rad/sec of its velocity setpoint.
   */
  public boolean atTargetVelocity() {
    return flywheelAtSetpoint;
  }

  /**
   * Commands the shooter to pass into this robot's alliance zone.
   *
   * @param distanceToZone The distance from the robot to the alliance zone.
   */
  public void pass(double distanceToZone) {
    setHoodAngle(HOOD_MAX_ANGLE);

    AngularVelocity targetVelocity = PASSING_FLYWHEEL_VELOCITY_MAP.get(distanceToZone);
    setFlywheelVelocity(targetVelocity, FLYWHEEL_SHOOTING_SLOT);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    flywheelAtSetpoint =
        inputs.flywheelLeaderVelocity.isNear(flywheelSetpoint, FLYWHEEL_VELOCITY_TOLERANCE);
    hoodAtSetpoint = inputs.hoodPosition.isNear(hoodSetpoint, HOOD_TOLERANCE);

    Logger.recordOutput("Shooter/Hood Setpoint", hoodSetpoint);
    Logger.recordOutput("Shooter/Hood At Setpoint", hoodAtSetpoint);
    Logger.recordOutput("Shooter/Flywheel Setpoint", flywheelSetpoint);
    Logger.recordOutput("Shooter/Flywheel At Setpoint", flywheelAtSetpoint);

    LoggedTracer.record("Shooter periodic");
  }

  /**
   * Sets the target velocity of the flywheel.
   *
   * @param velocity The velocity setpoint.
   */
  private void setFlywheelVelocity(AngularVelocity velocity, int slot) {
    io.setFlywheelVelocity(velocity, slot);
    flywheelSetpoint = velocity;
  }

  /**
   * Sets the target position of the hood.
   *
   * @param angle The hood setpoint.
   */
  private void setHoodAngle(Angle angle) {
    io.setHoodAngle(angle);
    hoodSetpoint = angle;
  }

  @Override
  public void simulationPeriodic() {
    final Angle shooterAngle = inputs.hoodPosition;
    final double shooterAngleRads = shooterAngle.div(HOOD_MAX_ANGLE).magnitude() * HOOD_SIM_MAX_ANGLE.in(Radians);

    final Rotation3d rot = new Rotation3d(0, shooterAngleRads, 0);
    final Translation3d translation = ROBOT_TO_HOOD_AXLE.plus(HOOD_AXLE_TO_HOOD.rotateBy(rot));

    final Rotation3d flippedRot = new Rotation3d(0, -shooterAngleRads, 0);
    final Pose3d hoodPose = new Pose3d(translation, flippedRot);

    Logger.recordOutput("Shooter/HoodMechanismPosition", hoodPose);

    MapleSimManager.getInstance()
        .setShooterState(
            translation.plus(HOOD_SIM_SHOOTING_OFFSET),
            inputs.flywheelLeaderVelocity,
            NINETY_DEGREES.minus(inputs.hoodPosition));
  }

  /**
   * Commands the shooter to shoot into the hub.
   *
   * @param distanceToHub The distance from the robot to the hub.
   */
  public void shootIntoHub(double distanceToHub) {
    Angle hoodAngle = HOOD_ANGLE_MAP.get(distanceToHub);
    AngularVelocity targetVelocity = HUB_FLYWHEEL_VELOCITY_MAP.get(distanceToHub);

    setHoodAngle(hoodAngle);
    setFlywheelVelocity(targetVelocity, FLYWHEEL_SHOOTING_SLOT);
  }

  public void spinUpForHubShot(double distanceToHub) {
    final Angle angle = HOOD_ANGLE_MAP.get(distanceToHub);
    setHoodAngle(angle);

    setFlywheelVelocity(FLYWHEEL_IDLE_SPEED, FLYWHEEL_SPINUP_SLOT);
  }

  /** Commands the flywheel to coast out to conserve battery voltage. */
  public void stopFlywheel() {
    io.coastFlywheel();
  }

  /** Commands the hood to stow when it isn't being used. */
  public void stowHood() {
    setHoodAngle(HOOD_STOW_ANGLE);
  }
}
