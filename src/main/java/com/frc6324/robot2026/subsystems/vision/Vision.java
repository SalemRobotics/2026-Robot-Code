package com.frc6324.robot2026.subsystems.vision;

import static com.frc6324.robot2026.subsystems.vision.VisionConstants.CAMERA_NAMES;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.frc6324.robot2026.subsystems.vision.VisionIO.VisionEstimation;

public class Vision extends SubsystemBase {
  private final String[] names;
  private final VisionIO[] io;
  private final VisionInputsAutoLogged[] inputs;

  private final ArrayList<Pose2d> allRobotPoses = new ArrayList<>();
  private final ArrayList<Pose2d> allRobotPosesAccepted = new ArrayList<>();
  private final ArrayList<Pose2d> allRobotPosesRejected = new ArrayList<>();

  private final ArrayList<Pose2d> robotPoses = new ArrayList<>();
  private final ArrayList<Pose2d> robotPosesAccepted = new ArrayList<>();
  private final ArrayList<Pose2d> robotPosesRejected = new ArrayList<>();

  public Vision(VisionIO... io) {
    this.io = io;

    names = new String[io.length];
    inputs = new VisionInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      names[i] = CAMERA_NAMES[i];
      inputs[i] = new VisionInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();

    for (int i = 0; i < io.length; i++) {
      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();

      VisionInputsAutoLogged currentInputs = inputs[i];

      io[i].updateInputs(currentInputs);
      Logger.processInputs("Vision/" + names[i], currentInputs);

      for (VisionEstimation estimation : currentInputs.estimations) {
        // TODO: implement logic :\
      }

      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }
  }
}
