package com.frc6324.robot2026.subsystems.vision.apriltag;

import static com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagConstants.*;

import com.frc6324.lib.util.Lazy;
import com.frc6324.lib.util.PoseExtensions.PoseSupplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public final class AprilTagIOSim extends AprilTagIOPhoton {
  private static final Lazy<VisionSystemSim> system =
      new Lazy<>(() -> new VisionSystemSim("apriltag"));

  private final PhotonCameraSim cameraSim =
      new PhotonCameraSim(
          camera,
          new SimCameraProperties()
              .setCalibration(
                  CAMERA_RESOLUTIONS[index].width(),
                  CAMERA_RESOLUTIONS[index].height(),
                  CAMERA_FOVS[index])
              .setAvgLatencyMs(CAMERA_LATENCIES[index])
              .setFPS(CAMERA_FPS[index])
              .setExposureTimeMs(10));

  public AprilTagIOSim(OdometryPoseGetter odometryPoseGetter, PoseSupplier robotPoseGetter) {
    super(odometryPoseGetter);

    if (!system.isInitialized()) {
      VisionSystemSim sim = system.get();

      sim.addAprilTags(APRILTAG_LAYOUT);
      VisionUpdateThread.addCallback(() -> sim.update(robotPoseGetter.getPose()));
    }

    system.get().addCamera(cameraSim, ROBOT_TO_CAMERAS[index]);

    // Disable camera streaming for performance
    cameraSim.enableDrawWireframe(false);
    cameraSim.enableProcessedStream(false);
    cameraSim.enableRawStream(false);
  }
}
