package com.frc6324.robot2026.subsystems.vision.apriltag;

import static com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagConstants.*;

import com.frc6324.lib.util.Lazy;
import com.frc6324.robot2026.RobotState;
import org.photonvision.simulation.*;

public final class AprilTagIOSim extends AprilTagIOPhoton {
  private static final Lazy<VisionSystemSim> system =
      new Lazy<>(() -> new VisionSystemSim("apriltag"));

  private final PhotonCameraSim cameraSim =
      new PhotonCameraSim(
          camera,
          new SimCameraProperties()
              .setCalibration(
                  props.resolution().width(), props.resolution().height(), props.horizontalFOV())
              .setAvgLatencyMs(props.averageLatency())
              .setFPS(props.FPS())
              .setExposureTimeMs(10));

  public AprilTagIOSim() {
    if (!system.isInitialized()) {
      VisionSystemSim sim = system.get();

      sim.addAprilTags(APRILTAG_LAYOUT);
      VisionUpdateThread.addCallback(() -> sim.update(RobotState.getInstance().getPose()));
    }

    system.get().addCamera(cameraSim, props.robotToCameraLens());

    // Disable camera streaming for performance
    cameraSim.enableDrawWireframe(false);
    cameraSim.enableProcessedStream(false);
    cameraSim.enableRawStream(false);
  }
}
