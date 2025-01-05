package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for a vision IO */
public interface VisionIO {
  /** The inputs for a vision system */
  @AutoLog
  class VisionInputs {
    boolean hasResults = false;

    /** Ambiguity ration of the target, 0 is no ambiguity (good), 1 is maximum ambiguity (bad) */
    double ambiguityRatio = 0;

    int[] visibleTagIDs = {};
    Pose3d[] fieldSpaceRobotPoses = {};
    double timeStamp = 0;

    /** Area of the tags as a percentage of the screen taken up by the tag (0 - 100) */
    double[] tagAreas = {};

    String cameraName = "";
  }

  /**
   * Update the inputs of a vision system
   *
   * @param inputs the inputs to update
   */
  void updateInputs(VisionInputs inputs);

  /**
   * Pass the robot rotation that is measured with the IMU to the vision system This should be updated every loop
   *
   * @param robotRotation Actual rotation of the robot
   */
  default void setRobotRotation(Rotation2d robotRotation) {}
}