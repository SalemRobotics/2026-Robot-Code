package com.frc6324.robot2026.subsystems.drive;

import static com.frc6324.robot2026.subsystems.drive.DrivetrainConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.frc6324.robot2026.generated.TunerConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public sealed class DriveIOCTRE extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements DriveIO permits DriveIOSim {
  private final StatusSignalCollection gyroscopeSignals;
  private final StatusSignal<AngularVelocity> pitchVelocitySignal;
  private final StatusSignal<AngularVelocity> rollVelocitySignal;
  private final StatusSignal<Angle> rollSignal;
  private final StatusSignal<Angle> pitchSignal;
  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;

  private boolean hasAppliedOperatorPerspective = false;

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
      super.resetPose(SIM_STARTING_POSE);
    }

    Pigeon2 pigeon = getPigeon2();

    // Store signals from the pigeon we care about
    pitchVelocitySignal = pigeon.getAngularVelocityYWorld();
    rollVelocitySignal = pigeon.getAngularVelocityXWorld();
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
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  @Override
  public void logModuleStates(SwerveDriveState state) {
    // Stop if the module states or targets are null so we don't cause an NPE
    if (state.ModuleStates == null || state.ModuleTargets == null) {
      return;
    }

    for (int i = 0; i < 4; i++) {
      // Get the current module and its name
      final SwerveModule<?, ?, CANcoder> module = getModule(i);
      final String name = MODULE_NAMES[i];

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
  public void updateInputs(final DriveInputs inputs) {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);
                hasAppliedOperatorPerspective = true;
              });
    }

    final SwerveDriveState state = getState();

    // Update the last known odometry
    DrivingUtils.updateOdometry(state.Pose, state.Speeds);

    // Copy the recorded state into the inputs
    inputs.copyFromState(state);

    // Rip the gyro angle straight from the pigeon
    inputs.GyroAngle = getPigeon2().getRotation2d();

    // Refresh all of the status signals
    gyroscopeSignals.refreshAll();

    // Get all of the other gyro values we care about
    final Angle roll = rollSignal.getValue();
    final Angle pitch = pitchSignal.getValue();
    final AngularVelocity rollVelocity = rollVelocitySignal.getValue();
    final AngularVelocity pitchVelocity = pitchVelocitySignal.getValue();
    final LinearAcceleration accelX = accelerationX.getValue();
    final LinearAcceleration accelY = accelerationY.getValue();

    // Send all of the tilt values to the driving safety util
    DrivingUtils.updateTilt(roll, pitch, rollVelocity, pitchVelocity, accelX, accelY);
  }
}
