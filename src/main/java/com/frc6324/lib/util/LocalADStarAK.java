/*
 * Copyright (c) 2025 The Blue Devils.
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */
package com.frc6324.lib.util;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.LocalADStar;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LocalADStarAK extends LocalADStar implements LoggableInputs {
  private boolean isNewPathAvailable = false;
  private List<PathPoint> currentPathPoints = Collections.emptyList();

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  @Override
  public boolean isNewPathAvailable() {
    if (!Logger.hasReplaySource()) {
      isNewPathAvailable = super.isNewPathAvailable();
    }

    Logger.processInputs("LocalADStarAK", this);

    return isNewPathAvailable;
  }

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    if (!Logger.hasReplaySource()) {
      PathPlannerPath currentPath = super.getCurrentPath(constraints, goalEndState);

      if (currentPath != null) {
        currentPathPoints = currentPath.getAllPathPoints();
      } else {
        currentPathPoints = Collections.emptyList();
      }
    }

    Logger.processInputs("LocalADStarAK", this);

    if (currentPathPoints.isEmpty()) {
      return null;
    }

    return PathPlannerPath.fromPathPoints(currentPathPoints, constraints, goalEndState);
  }

  @Override
  public void setStartPosition(Translation2d startPosition) {
    if (!Logger.hasReplaySource()) {
      super.setStartPosition(startPosition);
    }
  }

  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    if (!Logger.hasReplaySource()) {
      super.setGoalPosition(goalPosition);
    }
  }

  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    if (!Logger.hasReplaySource()) {
      super.setDynamicObstacles(obs, currentRobotPos);
    }
  }

  @Override
  public void toLog(LogTable table) {
    table.put("IsNewPathAvailable", isNewPathAvailable);

    double[] pointsLogged = new double[currentPathPoints.size() * 2];
    int idx = 0;
    for (PathPoint point : currentPathPoints) {
      pointsLogged[idx] = point.position.getX();
      pointsLogged[idx + 1] = point.position.getY();
      idx += 2;
    }

    table.put("CurrentPathPoints", pointsLogged);
  }

  @Override
  @SuppressWarnings("nullness")
  public void fromLog(LogTable table) {
    isNewPathAvailable = table.get("IsNewPathAvailable", false);

    double[] pointsLogged = table.get("CurrentPathPoints", new double[0]);

    List<PathPoint> pathPoints = new ArrayList<>();
    for (int i = 0; i < pointsLogged.length; i += 2) {
      pathPoints.add(new PathPoint(new Translation2d(pointsLogged[i], pointsLogged[i + 1]), null));
    }

    currentPathPoints = pathPoints;
  }
}
