package com.frc6324.robot2026.subsystems.shooter;

import static com.frc6324.lib.util.CommonUtils.NINETY_DEGREES;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.*;
import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.DrumConstants.*;
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
  private AngularVelocity drumSetpoint = RadiansPerSecond.zero();
  private boolean drumAtSetpoint = false;

  private AngularVelocity velocityOffset = RotationsPerSecond.zero();

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

  public void decrementOffset() {
    velocityOffset = velocityOffset.minus(DRUM_OFFSET_STEP);
  }

  public boolean drumAtSpeed() {
    return drumAtSetpoint;
  }

  public void incrementOffset() {
    velocityOffset = velocityOffset.plus(DRUM_OFFSET_STEP);
  }

  /**
   * Commands the shooter to pass into this robot's alliance zone.
   *
   * @param distanceToZone The distance from the robot to the alliance zone.
   */
  public void pass(double distanceToZone) {
    setHoodAngle(HOOD_MAX_ANGLE);

    AngularVelocity targetVelocity = PASSING_VELOCITY_MAP.get(distanceToZone);
    setDrumVelocity(targetVelocity, DRUM_SHOOTING_SLOT);
  }

  @Override
  public void periodic() {
    // Update the shooter inputs and log them
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    hoodAtSetpoint = inputs.hoodPosition.isNear(hoodSetpoint, HOOD_POSITION_TOLERANCE);
    drumAtSetpoint = inputs.drumVelocity.isNear(drumSetpoint, DRUM_VELOCITY_TOLERANCE);

    Logger.recordOutput("Shooter/Hood Setpoint", hoodSetpoint);
    Logger.recordOutput("Shooter/Flywheel Setpoint", drumSetpoint);
    Logger.recordOutput("Shooter/Velocity Offset", velocityOffset);

    LoggedTracer.record("Shooter periodic");
  }

  public void resetOffset() {
    velocityOffset = RotationsPerSecond.zero();
  }

  /**
   * Sets the target velocity of the flywheel.
   *
   * @param velocity The velocity setpoint.
   */
  private void setDrumVelocity(AngularVelocity velocity, int slot) {
    io.setDrumVelocity(velocity.plus(velocityOffset), slot);
    io.setAcceleratorVelocity(velocity);
    drumSetpoint = velocity;
  }

  /**
   * Sets the target position of the hood.
   *
   * @param angle The hood setpoint.
   */
  private void setHoodAngle(Angle angle) {
    io.setHoodPosition(angle);
    hoodSetpoint = angle;
  }

  @Override
  public void simulationPeriodic() {
    final Angle shooterAngle = inputs.hoodPosition;
    final double shooterAngleRads =
        shooterAngle.div(HOOD_MAX_ANGLE).magnitude() * HOOD_SIM_MAX_ANGLE.in(Radians);

    final Rotation3d rot = new Rotation3d(0, shooterAngleRads, 0);
    final Translation3d translation = ROBOT_TO_HOOD_AXLE.plus(HOOD_AXLE_TO_HOOD.rotateBy(rot));

    final Rotation3d flippedRot = new Rotation3d(0, -shooterAngleRads, 0);
    final Pose3d hoodPose = new Pose3d(translation, flippedRot);

    Logger.recordOutput("Shooter/HoodMechanismPosition", hoodPose);

    MapleSimManager.getInstance()
        .setShooterState(
            translation.plus(HOOD_SIM_SHOOTING_OFFSET),
            inputs.drumVelocity,
            NINETY_DEGREES.minus(inputs.hoodPosition));
  }

  /**
   * Commands the shooter to shoot into the hub.
   *
   * @param distanceToHub The distance from the robot to the hub.
   */
  public void shootIntoHub(double distanceToHub) {
    final HubShotParams params = HUB_SHOT_MAP.get(distanceToHub);

    if (params == null) {
      return;
    }

    setHoodAngle(params.hoodAngle());
    setDrumVelocity(params.drumVelocity(), DRUM_SHOOTING_SLOT);
  }

  public void shootUpAgainstHub() {
    setHoodAngle(HOOD_STOW_ANGLE);
    setDrumVelocity(DRUM_CLOSE_HUB_SHOT_SPEED, DRUM_SHOOTING_SLOT);
  }

  public void spinUpForHubShot(double distanceToHub) {
    final HubShotParams params = HUB_SHOT_MAP.get(distanceToHub);
    if (params == null) {
      return;
    }

    setHoodAngle(params.hoodAngle());
    setDrumVelocity(DRUM_IDLE_SPEED, DRUM_SPINUP_SLOT);
  }

  public void stopAccelerators() {
    io.stopAcceleratorMotor();
  }

  /** Commands the drum to coast out to conserve battery voltage. */
  public void stopDrum() {
    io.coastDrum();
  }

  /** Commands the hood to stow when it isn't being used. */
  public void stowHood() {
    setHoodAngle(HOOD_STOW_ANGLE);
  }
}
