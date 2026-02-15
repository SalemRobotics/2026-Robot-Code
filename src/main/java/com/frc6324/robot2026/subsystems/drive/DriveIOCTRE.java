package com.frc6324.robot2026.subsystems.drive;

import static com.frc6324.robot2026.subsystems.drive.DrivetrainConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.frc6324.robot2026.generated.TunerConstants;
import com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagIOPhoton.OdometryPoseGetter;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public sealed class DriveIOCTRE extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements DriveIO, OdometryPoseGetter permits DriveIOSim {
  private final StatusSignalCollection gyroscopeSignals;
  private final StatusSignal<AngularVelocity> pitchVelocitySignal;
  private final StatusSignal<AngularVelocity> rollVelocitySignal;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<Angle> rollSignal;
  private final StatusSignal<Angle> pitchSignal;
  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;
  private final SwerveDriveState state;

  public DriveIOCTRE() {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        TunerConstants.DrivetrainConstants,
        ODOMETRY_UPDATE_FREQUENCY,
        ODOMETRY_STDDEVS,
        DEFAULT_VISION_STDDEVS,
        SwerveDrive.regulateModuleConstantsForSimulation(
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight));

    // Reset the pose in simulation to somewhere known
    if (RobotBase.isSimulation()) {
      super.resetPose(STARTING_POSE);
    }

    // Get the state and pigeon of the drivetrain
    state = getState();
    Pigeon2 pigeon = getPigeon2();

    // Store signals from the pigeon we care about
    pitchVelocitySignal = pigeon.getAngularVelocityYWorld();
    rollVelocitySignal = pigeon.getAngularVelocityXWorld();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    rollSignal = pigeon.getRoll();
    pitchSignal = pigeon.getPitch();
    accelerationX = pigeon.getAccelerationX();
    accelerationY = pigeon.getAccelerationY();

    // Create the signal collection
    gyroscopeSignals =
        new StatusSignalCollection(
            pitchSignal, pitchVelocitySignal,
            rollSignal, rollVelocitySignal,
            accelerationX, accelerationY);

    // Set signal update frequencies
    gyroscopeSignals.setUpdateFrequencyForAll(100);
  }

  @Override
  public void logModuleStates(SwerveDriveState state) {
    // Stop if the module states or targets are null so we don't cause an NPE
    if (state.ModuleStates == null || state.ModuleTargets == null) {
      return;
    }

    for (int i = 0; i < 4; i++) {
      // Get the current module and its name
      SwerveModule<?, ?, CANcoder> module = getModule(i);
      String name = MODULE_NAMES[i];

      // Log steering information
      Logger.recordOutput(
          "Drive/" + name + "/Absolute Encoder Angle",
          module.getEncoder().getAbsolutePosition().getValue());
      Logger.recordOutput("Drive/" + name + "/Steering Angle", state.ModuleStates[i].angle);
      Logger.recordOutput("Drive/" + name + "/Target Steering Angle", state.ModuleTargets[i].angle);
      // Log drive velocity information
      Logger.recordOutput(
          "Drive/" + name + "/Drive Velocity", state.ModuleStates[i].speedMetersPerSecond);
      Logger.recordOutput(
          "Drive/" + name + "/Target Drive Velocity", state.ModuleTargets[i].speedMetersPerSecond);
    }
  }

  @Override
  public void updateInputs(DriveInputs inputs) {
    // Update the last known odometry
    DrivingUtils.updateOdometry(state.Pose, state.Speeds);

    // Copy the recorded state into the inputs
    inputs.copyFromState(state);

    // Rip the gyro angle straight from the pigeon
    inputs.GyroAngle = getPigeon2().getRotation2d();

    // Refresh all of the status signals
    gyroscopeSignals.refreshAll();

    // Record states we don't care about for the safety below
    inputs.YawVelocity = yawVelocity.getValue();

    // Get all of the other gyro values we care about
    Angle roll = rollSignal.getValue();
    Angle pitch = pitchSignal.getValue();
    AngularVelocity rollVelocity = rollVelocitySignal.getValue();
    AngularVelocity pitchVelocity = pitchVelocitySignal.getValue();
    LinearAcceleration accelX = accelerationX.getValue();
    LinearAcceleration accelY = accelerationY.getValue();

    // Send all of the tilt values to the driving safety util
    DrivingUtils.updateTilt(roll, pitch, rollVelocity, pitchVelocity, accelX, accelY);

    // Finally, write all of the tils values into the inputs.
    inputs.Roll = roll;
    inputs.RollVelocity = rollVelocity;
    inputs.Pitch = pitch;
    inputs.PitchVelocity = pitchVelocity;
    inputs.AccelerationX = accelX;
    inputs.AccelerationY = accelY;
  }
}
