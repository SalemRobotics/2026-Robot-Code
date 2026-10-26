package com.frc6324.robot2026.subsystems.vision.apriltag;

import com.frc6324.lib.util.FieldConstants;
import com.frc6324.robot2026.subsystems.vision.apriltag.AprilTagIO.VisionEstimation;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Represents a rule that evaluates vision observations. */
public abstract class VisionRule {
  private final ArrayList<VisionEstimation> estimationsRejected = new ArrayList<>();
  public final String name;

  /** Creates a new VisionRule. Uses the subclass' name as the name for this rule. */
  protected VisionRule() {
    name = this.getClass().getSimpleName();
  }

  /** Gets the AdvantageKit log key for this vision rule. */
  protected final String logKey() {
    return "Vision/AprilTag/Rules/" + name;
  }

  public void logRejected() {
    Logger.recordOutput(
        logKey() + "/Rejected", estimationsRejected.toArray(VisionEstimation[]::new));

    Logger.recordOutput(logKey() + "/RejectedAny", estimationsRejected.size() > 0);
    estimationsRejected.clear();
  }

  public void reject(VisionEstimation estimation) {
    estimationsRejected.add(estimation);
  }

  public abstract boolean shouldPermit(VisionEstimation estimation);

  public void update() {}

  /** Checks if the vision measurement has an acceptable ambiguity, given the number of tags. */
  public static class AmbiguityFilter extends VisionRule {
    private static final double SINGLE_TAG_AMBIGUITY_THRESH = 0.25;
    private static final double MULTI_TAG_AMBIGUITY_THRESH = 0.4;

    public AmbiguityFilter() {
      Logger.recordOutput(logKey() + "/SingleTagThresh", SINGLE_TAG_AMBIGUITY_THRESH);
      Logger.recordOutput(logKey() + "/MultiTagThresh", MULTI_TAG_AMBIGUITY_THRESH);
    }

    @Override
    public boolean shouldPermit(VisionEstimation estimation) {
      if (estimation.numTagsUsed() == 0) {
        // Never permit estimations with 0 tags
        // (which should be impossible to have)
        return false;
      } else if (estimation.numTagsUsed() == 1) {
        return estimation.ambiguity() < SINGLE_TAG_AMBIGUITY_THRESH;
      } else {
        return estimation.ambiguity() < MULTI_TAG_AMBIGUITY_THRESH;
      }
    }
  }

  /**
   * Checks that the robot isn't spinning too quickly so that vision measurements aren't blurred to
   * the point of ruining accuracy.
   */
  public static class AngularVelocityFilter extends VisionRule {
    private static final double ANGULAR_VELOCITY_LIMIT = 1.5;

    private final Supplier<ChassisSpeeds> robotSpeedsSupplier;
    private ChassisSpeeds speeds = new ChassisSpeeds();

    public AngularVelocityFilter(Supplier<ChassisSpeeds> robotSpeedsSupplier) {
      this.robotSpeedsSupplier = robotSpeedsSupplier;

      Logger.recordOutput(logKey() + "/AngularVelocityLimit", ANGULAR_VELOCITY_LIMIT);
    }

    @Override
    public void update() {
      super.update();

      speeds = robotSpeedsSupplier.get();
    }

    @Override
    public boolean shouldPermit(VisionEstimation estimation) {
      return speeds.omegaRadiansPerSecond < ANGULAR_VELOCITY_LIMIT;
    }
  }

  /** Checks if the measurement is within an acceptable distance from the robot. */
  public static class DistanceFilter extends VisionRule {
    // A single
    private static final double DISTANCE_THRESHOLD = 8;

    public DistanceFilter() {
      Logger.recordOutput(logKey() + "/MaxDistance", DISTANCE_THRESHOLD);
    }

    @Override
    public boolean shouldPermit(VisionEstimation estimation) {
      if (estimation.numTagsUsed() == 1) {
        return estimation.averageTagDistance() < DISTANCE_THRESHOLD;
      } else {
        // TODO: determine if we want to filter distance for multitag
      }

      return true;
    }
  }

  /**
   * Checks if the pose is within the field borders and is otherwise possible for the robot to
   * achieve.
   */
  public static class WithinFieldBoundsFilter extends VisionRule {
    @Override
    public boolean shouldPermit(VisionEstimation estimation) {
      final Translation3d poseTranslation = estimation.robotPose().getTranslation();

      final boolean poseWithinX =
          (poseTranslation.getX() > 0) && (poseTranslation.getX() < FieldConstants.FIELD_LENGTH);
      final boolean poseWithinY =
          (poseTranslation.getY() > 0) && (poseTranslation.getY() < FieldConstants.FIELD_WIDTH);
      final boolean poseOnGround = poseTranslation.getZ() < 0.4;

      return poseWithinX && poseWithinY && poseOnGround;
    }
  }
}
