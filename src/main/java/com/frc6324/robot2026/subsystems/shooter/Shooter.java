package com.frc6324.robot2026.subsystems.shooter;

import static com.frc6324.robot2026.subsystems.shooter.ShooterConstants.*;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void pass() {
    io.setHoodAngle(SHOOTER_HOOD_MAX_ANGLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  @Override
  public void simulationPeriodic() {
    Rotation3d rot = new Rotation3d(0, inputs.hoodPosition.in(Radians), 0);
    Translation3d translation = ROBOT_TO_HOOD_AXLE.plus(HOOD_AXLE_TO_HOOD.rotateBy(rot));

    Pose3d hoodPose =
        new Pose3d(translation, new Rotation3d(0, Math.PI + inputs.hoodPosition.in(Radians), 0));

    Logger.recordOutput("Shooter/HoodPosition", hoodPose);
  }

  public void shootIntoHub(double distanceToHub) {}

  public void stop() {}

  public void stowHood() {
    io.setHoodAngle(SHOOTER_HOOD_STOW_ANGLE);
  }
}
