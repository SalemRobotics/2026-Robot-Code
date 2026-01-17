package com.frc6324.robot2026.subsystems.vision.apriltag;

import static com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagConstants.UPDATE_THREAD_FREQUENCY;

import com.frc6324.lib.UninstantiableClass;
import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;

@UninstantiableClass
public final class VisionUpdateThread {
  private VisionUpdateThread() {
    throw new IllegalAccessError();
  }

  private static final ArrayList<Runnable> updateFunctions = new ArrayList<>();
  private static final Notifier updateThread =
      new Notifier(() -> updateFunctions.forEach(Runnable::run));

  static {
    updateThread.setName("Vision Update Thread");
  }

  /**
   * Adds an update callback to be polled by this thread.
   *
   * @param callback The callback to add.
   */
  static void addCallback(Runnable callback) {
    updateFunctions.add(callback);
  }

  /** Starts the vision thread. */
  static void start() {
    updateThread.startPeriodic(UPDATE_THREAD_FREQUENCY);
  }
}
