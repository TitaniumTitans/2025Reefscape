package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
  class VisionInputs {
    boolean hasResults = false;

    // Ambiguity of the target, 0 is good, 1 is bad
    double ambiguityRatio = 0;

    int[] visableTagIDs = {};
    Pose3d[] fieldSpaceRobotPoses = {};
    double timestamp = 0;

    // percentage of the screen being taken up by the tags
    double[] tagAreas = {};

    String cameraName = "";
  }

  // update the vision inputs
  void updateInputs(VisionInputs inputs);

  // sets the robot actual rotation, must be updated each match
  default void setRobotRotation(Rotation2d robotRotation) {}
}
