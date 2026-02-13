package com.frc6324.robot2026.sim;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.List;
import org.ironmaple.simulation.gamepieces.GamePiece;

public final class FlyingFuelSimulation implements GamePiece {
  private static final double EPSILON = 1e-6;
  private static final double GRAVITY = 9.80665;
  private static final Translation3d GRAVITY_ACCEL_VECTOR = new Translation3d(0, 0, -GRAVITY);
  private static final double AIR_DENSITY = 1.225;
  private static final double DRAG_COEFFICIENT = 0.75;
  private static final double MASS_KG = Units.lbsToKilograms(0.5);
  private static final double RADIUS = 0.15 / 2; // Diameter is 15cm
  private static final double AREA = Math.PI * Math.pow(RADIUS, 2);

  private static final double DRAG_K = (0.5 * AIR_DENSITY * DRAG_COEFFICIENT * AREA) / MASS_KG;

  private Translation3d velocity;
  private Pose3d currentPosition;

  public FlyingFuelSimulation(
      Pose2d robotPose,
      Translation3d shooterPosition,
      Rotation2d shooterRotation,
      ChassisSpeeds drivetrainSpeeds,
      Angle shooterAngle,
      LinearVelocity launchSpeed) {
    final Translation3d forward =
        new Translation3d(1, 0, 0)
            .rotateBy(new Rotation3d(0, -shooterAngle.in(Radians), 0))
            .rotateBy(new Rotation3d(robotPose.getRotation().plus(shooterRotation)));

    final Translation2d chassisVelocity =
        new Translation2d(drivetrainSpeeds.vxMetersPerSecond, drivetrainSpeeds.vyMetersPerSecond);

    final Translation2d shooterVelocity =
        shooterPosition
            .toTranslation2d()
            .rotateBy(robotPose.getRotation())
            .rotateBy(Rotation2d.fromDegrees(90))
            .times(drivetrainSpeeds.omegaRadiansPerSecond)
            .plus(chassisVelocity);

    velocity =
        forward.times(launchSpeed.in(MetersPerSecond)).plus(new Translation3d(shooterVelocity));

    currentPosition =
        new Pose3d(
            new Translation3d(robotPose.getTranslation())
                .plus(shooterPosition.rotateBy(new Rotation3d(robotPose.getRotation()))),
            new Rotation3d(robotPose.getRotation().plus(shooterRotation)));
  }

  public void update(final List<FlyingFuelSimulation> toRemove, final double deltaTime) {
    if (currentPosition.getZ() < RADIUS + EPSILON) {
      toRemove.add(this);
      return;
    }

    final double speed = velocity.getNorm();

    final Translation3d dragAccel =
        speed < EPSILON ? Translation3d.kZero : velocity.div(speed).times(-DRAG_K * speed * speed);

    final Translation3d dv = dragAccel.plus(GRAVITY_ACCEL_VECTOR).times(deltaTime);
    velocity = velocity.plus(dv);

    currentPosition =
        new Pose3d(
            currentPosition.getTranslation().plus(velocity.times(deltaTime)),
            currentPosition.getRotation());
  }

  @Override
  public boolean isGrounded() {
    return false;
  }

  @Override
  public Pose3d getPose3d() {
    return currentPosition;
  }

  @Override
  public String getType() {
    return "Fuel";
  }

  @Override
  public Translation3d getVelocity3dMPS() {
    return velocity;
  }

  @Override
  public void triggerHitTargeCallBack() {}
}
