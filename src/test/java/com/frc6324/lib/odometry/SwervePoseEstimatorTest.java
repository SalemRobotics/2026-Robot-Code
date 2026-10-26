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
package com.frc6324.lib.odometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class SwervePoseEstimatorTest {
  private static final double EPSILON = 1e-9;

  private static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(0.5, 0.5),
          new Translation2d(0.5, -0.5),
          new Translation2d(-0.5, 0.5),
          new Translation2d(-0.5, -0.5));

  private static SwerveModulePosition[] positions(double distanceMeters) {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(distanceMeters, Rotation2d.kZero),
      new SwerveModulePosition(distanceMeters, Rotation2d.kZero),
      new SwerveModulePosition(distanceMeters, Rotation2d.kZero),
      new SwerveModulePosition(distanceMeters, Rotation2d.kZero)
    };
  }

  private static SwervePoseEstimator newEstimator() {
    return new SwervePoseEstimator(
        KINEMATICS,
        Rotation2d.kZero,
        positions(0.0),
        Pose2d.kZero,
        true,
        VecBuilder.fill(0.1, 0.1, 0.1));
  }

  @Test
  void replayRecomputesLaterMeasurementsAgainstTheUpdatedAnchor() {
    final SwervePoseEstimator estimator = newEstimator();
    estimator.updateWithTime(0.0, Optional.empty(), positions(0.0));
    estimator.updateWithTime(1.0, Optional.empty(), positions(0.0));
    estimator.updateWithTime(2.0, Optional.empty(), positions(0.0));

    final var stddevs = VecBuilder.fill(0.1, 0.1, 0.1);
    estimator.addVisionMeasurement(new Pose2d(1.0, 0.0, Rotation2d.kZero), 1.0, stddevs);
    estimator.addVisionMeasurement(new Pose2d(2.0, 0.0, Rotation2d.kZero), 2.0, stddevs);

    // Each measurement uses a gain of 0.5: 0.5 * 1.0 = 0.5, then 0.5 + 0.5 * (2.0 - 0.5) = 1.25.
    assertEquals(1.25, estimator.sampleAt(2.0).orElseThrow().getX(), EPSILON);

    // A late replacement at t=1 must replay the t=2 measurement relative to the new anchor.
    estimator.addVisionMeasurement(new Pose2d(4.0, 0.0, Rotation2d.kZero), 1.0, stddevs);

    // t=1: 0.5 + 0.5 * (4.0 - 0.5) = 2.25.
    assertEquals(2.25, estimator.sampleAt(1.0).orElseThrow().getX(), EPSILON);
    // t=2: 2.25 + 0.5 * (2.0 - 2.25) = 2.125.
    assertEquals(2.125, estimator.sampleAt(2.0).orElseThrow().getX(), EPSILON);
  }

  @Test
  void sampleSpeedsAtInterpolatesHistoricalOdometrySpeeds() {
    final SwervePoseEstimator estimator = newEstimator();
    estimator.updateWithTime(0.0, Optional.empty(), positions(0.0));
    estimator.updateWithTime(1.0, Optional.empty(), positions(1.0));

    final ChassisSpeeds speeds = estimator.sampleSpeedsAt(0.5).orElseThrow();
    assertEquals(0.5, speeds.vxMetersPerSecond, EPSILON);
    assertEquals(0.0, speeds.vyMetersPerSecond, EPSILON);
    assertEquals(0.0, speeds.omegaRadiansPerSecond, EPSILON);
  }

  @Test
  void sampleSpeedsAtReturnsEmptyBeforeAnyOdometry() {
    assertTrue(newEstimator().sampleSpeedsAt(0.0).isEmpty());
  }
}
