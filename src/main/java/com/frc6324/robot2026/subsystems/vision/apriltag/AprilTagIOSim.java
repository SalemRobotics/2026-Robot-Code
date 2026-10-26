package com.frc6324.robot2026.subsystems.vision.apriltag;

import static com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagConstants.*;

import com.frc6324.lib.util.Lazy;
import com.frc6324.robot2026.RobotState;
import org.photonvision.simulation.*;

public final class AprilTagIOSim extends AprilTagIOPhoton {
  private static final Lazy<VisionSystemSim> system =
      new Lazy<>(
          () -> {
            final VisionSystemSim sim = new VisionSystemSim("apriltag");
            sim.addAprilTags(APRILTAG_LAYOUT);

            VisionUpdateThread.addCallback(() -> sim.update(RobotState.getInstance().getPose()));
            return sim;
          });

  public AprilTagIOSim(CameraProps props) {
    super(props, system.get());

    final PhotonCameraSim cameraSim =
        new PhotonCameraSim(
            camera,
            new SimCameraProperties()
                .setCalibration(
                    props.resolution().width(), props.resolution().height(), props.horizontalFOV())
                .setAvgLatencyMs(props.averageLatency())
                .setFPS(props.FPS())
                .setExposureTimeMs(10));

    system.get().addCamera(cameraSim, props.robotToCameraLens());

    // Disable camera streaming for performance
    cameraSim.enableDrawWireframe(false);
    cameraSim.enableProcessedStream(false);
    cameraSim.enableRawStream(false);
  }
}
