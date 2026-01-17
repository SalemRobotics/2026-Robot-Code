package com.frc6324.robot2026.subsystems.vision.objdetect;

import static com.frc6324.robot2026.subsystems.vision.objdetect.ObjectDetectionConstants.*;

import com.frc6324.robot2026.subsystems.vision.objdetect.ObjDetectIO.ObjDetectInputs;
import com.frc6324.robot2026.subsystems.vision.objdetect.ObjDetectIO.VisibleGamePiece;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Comparator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public final class ObjectDetection extends SubsystemBase {
  private final ObjDetectIO[] io;
  private final ObjDetectInputsAutoLogged[] inputs;

  public ObjectDetection(ObjDetectIO... io) {
    this.io = io;
    inputs = new ObjDetectInputsAutoLogged[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new ObjDetectInputsAutoLogged();
    }
  }

  /**
   * Calculates the new "most optimal" path, that is the path at which we can intake the most game
   * pieces.
   */
  public double calculateOptimalPath() {
    // Create some lists so we don't have to keep the inputs locked for long
    final ArrayList<RangeEvent> events = new ArrayList<>();
    final ArrayList<VisibleGamePiece> allPieces = new ArrayList<>();

    // Get all of the game pieces from all of the cameras
    for (final ObjDetectInputs input : inputs) {
      // if the camera isn't connected, skip
      if (!input.connected) {
        continue;
      }

      for (final VisibleGamePiece piece : input.visiblePieces) {
        allPieces.add(piece);
      }
    }

    final int numPieces = allPieces.size();
    for (int i = 0; i < numPieces; i++) {
      final VisibleGamePiece piece = allPieces.get(i);

      // Get the angle range at which we can intake the piece
      final double start = piece.startTheta();
      final double end = piece.endTheta();

      if (start > end) {
        // Handle angles that cross the ±π boundary specially so that they don't break the algorithm

        // Add the range from start -> π
        events.add(new RangeEvent(i, start, 1));
        events.add(new RangeEvent(i, Math.PI, -1));

        // Add the range from -π -> end
        events.add(new RangeEvent(i, -Math.PI, 1));
        events.add(new RangeEvent(i, end, -1));
      } else {
        // Handle continuous angles in a regular way

        // Add the range from start -> end
        events.add(new RangeEvent(i, start, 1));
        events.add(new RangeEvent(i, end, -1));
      }
    }

    // Sort the events in ascending order (required for the algorithm to work)
    events.sort(Comparator.comparingDouble(e -> e.angle));

    // Keep track of details about the "best" range
    int bestCount = 0;
    double bestStart = 0;
    double bestEnd = 0;
    double bestDistance = 0;

    // Keep track of less info about the current range
    int currentCount = 0;
    double currentTotalDistance = 0;

    final int numEvents = events.size();
    for (int eventIndex = 0; eventIndex < numEvents; eventIndex++) {
      final RangeEvent event = events.get(eventIndex);

      // Add the delta (1 or -1, depending on whether we're starting or ending a piece's range)
      currentCount += event.delta;

      // Get the distance from the robot to the piece
      final double pieceDistance = allPieces.get(event.pieceIndex).distance();

      // If the delta is -1 (we're popping a piece), subtract its distance from the total, otherwise
      // add it
      if (event.delta == -1) {
        currentTotalDistance -= pieceDistance;
      } else {
        currentTotalDistance += pieceDistance;
      }

      // Compute the average distance
      final double currentAvgDistance = currentTotalDistance / currentCount;

      // If the current range has more pieces contained in it, or has a better average distance,
      // update the path
      if (currentCount > bestCount
          || (currentCount == bestCount && currentAvgDistance < bestDistance)) {
        final double next;
        final int nextIndex = eventIndex + 1;
        // If the next range doesn't exist, the range is simply the end angle
        if (nextIndex == numEvents) {
          next = event.angle;
        } else {
          // Otherwise, it's now from this angle to the next
          next = events.get(nextIndex).angle;
        }

        // Update the information about the best range
        bestCount = currentCount;
        bestStart = event.angle;
        bestEnd = next;
        bestDistance = currentAvgDistance;
      }
    }

    return (bestStart + bestEnd) / 2;
  }

  /**
   * Gets the angle at which the robot can best intake game pieces.
   *
   * @return The robot-relative target heading of the robot.
   */
  @AutoLogOutput(key = "Vision/Object Detection/Most Optimal Direction")
  public Rotation2d getMostOptimalHeadingRadians() {
    return Rotation2d.fromRadians(calculateOptimalPath());
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      ObjDetectInputsAutoLogged currentInputs = inputs[i];

      io[i].updateInputs(currentInputs);
      Logger.processInputs("Vision/Object Detection/" + CAMERA_NAMES[i], currentInputs);
    }
  }

  /**
   * Represents a simple "event" (the beginning or end of a game piece's range) for use in finding a
   * most optimal path.
   */
  record RangeEvent(int pieceIndex, double angle, int delta) {}
}
