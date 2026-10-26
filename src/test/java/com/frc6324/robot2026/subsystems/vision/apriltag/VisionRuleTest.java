/*
 * Copyright (c) 2026 The Blue Devils.
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
package com.frc6324.robot2026.subsystems.vision.apriltag;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.frc6324.lib.util.FieldConstants;
import com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagIO.EstimationStrategy;
import com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagIO.VisionEstimation;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.junit.jupiter.api.Test;

class VisionRuleTest {
  private static final VisionRule.WithinFieldBoundsFilter FIELD_BOUNDS_FILTER =
      new VisionRule.WithinFieldBoundsFilter();

  private static VisionEstimation estimationAt(double x, double y, double z) {
    return new VisionEstimation(
        new Pose3d(x, y, z, new Rotation3d()), 0.0, 0.0, 0.0, 1, EstimationStrategy.Multitag);
  }

  @Test
  void acceptsPoseOnFieldAtFloorLevel() {
    assertTrue(
        FIELD_BOUNDS_FILTER.shouldPermit(
            estimationAt(FieldConstants.FIELD_LENGTH / 2, FieldConstants.FIELD_WIDTH / 2, 0.0)));
  }

  @Test
  void acceptsPoseWithinGroundTolerance() {
    assertTrue(FIELD_BOUNDS_FILTER.shouldPermit(estimationAt(1.0, 1.0, -0.1)));
    assertTrue(FIELD_BOUNDS_FILTER.shouldPermit(estimationAt(1.0, 1.0, 0.3)));
  }

  @Test
  void rejectsPosesOutsideFieldBounds() {
    assertFalse(FIELD_BOUNDS_FILTER.shouldPermit(estimationAt(-1.0, 1.0, 0.0)));
    assertFalse(FIELD_BOUNDS_FILTER.shouldPermit(estimationAt(1.0, -1.0, 0.0)));
    assertFalse(
        FIELD_BOUNDS_FILTER.shouldPermit(
            estimationAt(FieldConstants.FIELD_LENGTH + 1.0, 1.0, 0.0)));
    assertFalse(
        FIELD_BOUNDS_FILTER.shouldPermit(estimationAt(1.0, FieldConstants.FIELD_WIDTH + 1.0, 0.0)));
  }

  @Test
  void rejectsPosesThatAreNotNearTheGround() {
    assertFalse(FIELD_BOUNDS_FILTER.shouldPermit(estimationAt(1.0, 1.0, 1.0)));
    assertFalse(FIELD_BOUNDS_FILTER.shouldPermit(estimationAt(1.0, 1.0, -1.0)));
  }
}
