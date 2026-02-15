package com.frc6324.robot2026.sim;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.function.Consumer;
import org.ironmaple.simulation.gamepieces.GamePiece;

/**
 * A simulation of a fuel flying through the air.
 *
 * <p>This is a direct replacement for {@link
 * org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly RebuiltFuelOnFly} that
 * improves upon the handling of gravity and adds drag to the fuel. You must use this class with
 * {@link RebuiltArena} via {@link RebuiltArena#addFuelProjectile(FlyingFuelSimulation)} because it
 * does not extend maple-sim's expected classes.
 */
public final class FlyingFuelSimulation implements GamePiece {
  /** The tolerance for floating-point error. */
  private static final double EPSILON = 1e-6;

  /** The gravitational constant, in meters per second squared */
  private static final double GRAVITY = 9.80665;

  /** The acceleration vector created by only gravity. */
  private static final Translation3d GRAVITY_ACCEL_VECTOR = new Translation3d(0, 0, -GRAVITY);

  /** The density of the air. */
  private static final double AIR_DENSITY = 1.225;

  // Fuel-specific constants
  /** The drag coefficient of the fuel. */
  private static final double DRAG_COEFFICIENT = 0.75;

  /** The mass of the fuel. */
  private static final double MASS_KG = Units.lbsToKilograms(0.5);

  /** The radius of the fuel. */
  private static final double RADIUS = 0.15 / 2; // Diameter is 15cm

  /** The surface area of the fuel. */
  private static final double AREA = Math.PI * Math.pow(RADIUS, 2);

  /**
   * The drag constant of a fuel, defined as {@code ((air density * drag coeff * area) / mass) / 2}.
   */
  private static final double DRAG_K = (0.5 * AIR_DENSITY * DRAG_COEFFICIENT * AREA) / MASS_KG;

  /** The fuel's current velocity. */
  private Translation3d velocity = Translation3d.kZero;

  /** The fuel's current position. */
  private Pose3d currentPosition = Pose3d.kZero;

  /**
   * Creates a new simulation of a fuel being shot out of a shooter mechanism.
   *
   * <p>For an unmoving shooter, {@code shooterRotation} should be a constant. For robots with a
   * turret, this should be dynamically computed given encoder values.
   *
   * <p>To increase realism, you may want to multiply {@code launchSpeed} by a "flywheel efficiency"
   * constant (typically 60-90%) to better account for things such as compression, slipping, etc.
   * that can't be simulated.
   *
   * @param robotPose The current 2D position of the robot.
   * @param shooterPosition The offset from the center of the robot to the center of the shooter.
   * @param shooterRotation The 2D rotational offset from the robot's "forward" to the direction the
   *     shooter shoots at.
   * @param drivetrainSpeeds The current speeds of the robot. It is perfectly OK to use {@code new
   *     ChassisSpeeds()} as this argument if you don't care about simulating drivetrain influence
   *     on the shooter.
   * @param shooterAngle The angle the shooter shoots at. An angle of 0° means that the shooter
   *     shoots parallel to the ground; likewise, an angle of 90° will shoot the fuel vertically.
   * @param launchSpeed The speed the fuel is being launched at. This should be calculated using
   *     some variation of {@code launcherWheelRadius * flywheelVelocity}.
   */
  public FlyingFuelSimulation(
      Pose2d robotPose,
      Translation3d shooterPosition,
      Rotation2d shooterRotation,
      ChassisSpeeds drivetrainSpeeds,
      Angle shooterAngle,
      LinearVelocity launchSpeed) {
    // Define the shooter's "forward" multiplier vector
    final Translation3d forward =
        new Translation3d(1, 0, 0)
            // Rotate the forward vector by the angle of the shooter
            // (this is negative because leaving it positive, according to WPIlib, shoots into the
            // ground!)
            .rotateBy(new Rotation3d(0, -shooterAngle.in(Radians), 0))
            // Rotate the forward vector by the robot's rotation and the shooter's rotational offset
            .rotateBy(new Rotation3d(robotPose.getRotation().plus(shooterRotation)));

    // Get the 2D velocity of the robot chassis
    final Translation2d chassisVelocity =
        new Translation2d(drivetrainSpeeds.vxMetersPerSecond, drivetrainSpeeds.vyMetersPerSecond);

    // Compute the velocity offset from robot movement
    final Translation2d shooterVelocity =
        shooterPosition
            .toTranslation2d()
            .rotateBy(robotPose.getRotation())
            .rotateBy(Rotation2d.fromDegrees(90))
            .times(drivetrainSpeeds.omegaRadiansPerSecond)
            .plus(chassisVelocity);

    // Add the launch velocity to the chassis velocity to arrive at an initial velocity
    velocity =
        forward.times(launchSpeed.in(MetersPerSecond)).plus(new Translation3d(shooterVelocity));

    // Derive the current position from the shooter's relative position to the robot's known
    // position
    currentPosition =
        new Pose3d(
            new Translation3d(robotPose.getTranslation())
                .plus(shooterPosition.rotateBy(new Rotation3d(robotPose.getRotation()))),
            new Rotation3d(robotPose.getRotation().plus(shooterRotation)));
  }

  @Override
  public Pose3d getPose3d() {
    // Return the current position
    return currentPosition;
  }

  @Override
  public String getType() {
    // This is always a fuel!
    return "Fuel";
  }

  @Override
  public Translation3d getVelocity3dMPS() {
    // Return the velocity vector
    return velocity;
  }

  @Override
  public boolean isGrounded() {
    // This piece flies, so return false
    return false;
  }

  @Override
  public void triggerHitTargeCallBack() {
    // We don't support any configurable callbacks, so this is empty
  }

  /**
   * Updates this simulation's position and velocity over a given period of time.
   *
   * <p>This method should only be called by an instance of {@link SimulatedArena} unless you know
   * what you're doing!
   *
   * @param hitFloorCallback The callback to call if this fuel hits the ground.
   * @param deltaTime The difference in time since this method was last called.
   */
  public void update(
      final Consumer<FlyingFuelSimulation> hitFloorCallback, final double deltaTime) {
    if (currentPosition.getZ() < RADIUS + EPSILON) {
      hitFloorCallback.accept(this);
      return;
    }

    // Calculate the total speed vector magnitude
    final double speed = velocity.getNorm();
    // Use the magnitude of speed and the drag constant to compute the air resistance experienced by
    // the fuel
    final Translation3d dragAccel =
        speed < EPSILON ? Translation3d.kZero : velocity.div(speed).times(-DRAG_K * speed * speed);

    // Compute the change in velocity since the last update() call
    final Translation3d dv = dragAccel.plus(GRAVITY_ACCEL_VECTOR).times(deltaTime);
    velocity = velocity.plus(dv);

    // Integrate the current position
    currentPosition =
        new Pose3d(
            currentPosition.getTranslation().plus(velocity.times(deltaTime)),
            currentPosition.getRotation());
  }
}
