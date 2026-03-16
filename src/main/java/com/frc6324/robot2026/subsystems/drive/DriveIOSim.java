package com.frc6324.robot2026.subsystems.drive;

import com.frc6324.robot2026.generated.TunerConstants;
import com.frc6324.robot2026.sim.MapleSimDriveBase;
import edu.wpi.first.math.geometry.Pose2d;
import lombok.Getter;

public final class DriveIOSim extends DriveIOCTRE {
  @Getter
  @SuppressWarnings("unchecked")
  private final MapleSimDriveBase driveSimulation =
      new MapleSimDriveBase(
          getPigeon2(),
          getModules(),
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);

  public DriveIOSim() {
    super();

    registerTelemetry(
        state -> {
          state.Pose = driveSimulation.getPose();
          state.Speeds = driveSimulation.getChassisSpeeds();
        });
  }

  @Override
  public void updateInputs(final DriveInputs inputs) {
    driveSimulation.update();
    super.updateInputs(inputs);
  }

  @Override
  public void resetPose(Pose2d pose) {
    driveSimulation.setPose(pose);

    super.resetPose(pose);
  }
}
