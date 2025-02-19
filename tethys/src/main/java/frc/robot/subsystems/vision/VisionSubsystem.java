package frc.robot.subsystems.vision;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import lombok.extern.java.Log;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {
  private final VisionIO[] visionIOs;
  private final VisionInputsAutoLogged[] inputs;
  private final VisionFilterParameters filterParameters;
  private Pose2d actualRobotPose = new Pose2d();
  public VisionSubsystem(VisionFilterParameters parameters, VisionIO... ios) {
    visionIOs = ios;
    filterParameters = parameters;

    // creates a new array of default inputs
    inputs = new VisionInputsAutoLogged[visionIOs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      inputs[i] = new VisionInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    // This is called once per scheduler run

    // update all cameras
    for (int i = 0; i < visionIOs.length; i++) {
      // update inputs
      visionIOs[i].updateInputs(inputs[i]);
      visionIOs[i].setRobotRotation(actualRobotPose.getRotation());

      Logger.processInputs("Vision/" + inputs[i].cameraName + " (" + i + ")", inputs[i]);
    }
  }

  public Command processVision(Supplier<Pose2d> poseSupplier) {
    return run(() -> {
      // update the real robot pose
      actualRobotPose = poseSupplier.get();
      List<Pose3d> tagPoses = new ArrayList<>();

      // get all cameras
      for (int i = 0; i < visionIOs.length; i++) {
        // check for bad or missing inputs
        if (!inputs[i].hasResults) {
          continue;
        }

        // if ambiguity is too high, skip
        if (inputs[i].ambiguityRatio > filterParameters.maxAmbiguityRatio()) {
          continue;
        }

        for (int tagId : inputs[i].visibleTagIDs) {
          Optional<Pose3d> tagPose = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)
              .getTagPose(tagId);
          tagPose.ifPresent(tagPoses::add);
        }

        // use the closest pose
        Pose3d closestPose = selectClosestPose(inputs[i].fieldSpaceRobotPoses, actualRobotPose);
        if (closestPose == null || outsideFieldBounds(closestPose)) {
          continue;
        }

        // Determine the standard deviation of the measurement using the distance.
        // The distance is estimated based on the tag area in percent (0-100)
        double tagDistance = calcuateAverageTagDistance(inputs[i].tagAreas);
        Matrix<N3, N1> stdDevMat =
                calculateStandardDeviations(tagDistance, inputs[i].fieldSpaceRobotPoses.length > 1);

        // Retrieve the timestamp of the measurement
        double timestamp = inputs[i].timeStamp;

        // Log the accepted pose
        Logger.recordOutput(
                "Vision/Accepted Poses/" + inputs[i].cameraName + " (" + i + ")", closestPose);

        RobotState.getInstance().addVisionMeasurement(
            new RobotState.VisionObservation(closestPose.toPose2d(), timestamp, stdDevMat)
        );
      }

      Logger.recordOutput("Vision/Visible Tag Poses", tagPoses.toArray(Pose3d[]::new));
    });
  }

  private Pose3d selectClosestPose(Pose3d[] fieldPoses, Pose2d actualRobotPose) {
    // Get the distance between the actual robot pose and each estimated pose
    double minDistance = Double.POSITIVE_INFINITY;
    Pose3d selectedPose = null;
    Pose3d actualPose = new Pose3d(actualRobotPose);

    for (Pose3d robotPose : fieldPoses) {
      double distance = robotPose.getTranslation().getDistance(actualPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        selectedPose = robotPose;
      }
    }

    return selectedPose;
  }

  private boolean outsideFieldBounds(Pose3d selectedPose) {
    return selectedPose.getTranslation().getX() < 0
            || selectedPose.getTranslation().getX()
            > filterParameters.fieldLength().in(Units.Meters)
            || selectedPose.getTranslation().getY() < 0
            || selectedPose.getTranslation().getY()
            > filterParameters.fieldWidth().in(Units.Meters)
            || selectedPose.getTranslation().getZ()
            < -filterParameters.zMargin().in(Units.Meters)
            || selectedPose.getTranslation().getZ()
            > filterParameters.zMargin().in(Units.Meters);
  }

  private double calcuateAverageTagDistance(double[] tagAreas) {
    double averageTagArea = 0.0;
    for (double area : tagAreas) {
      averageTagArea += area;
    }
    averageTagArea /= tagAreas.length;

    // estimate distance when the tag fills up %100 of the camera
    double minTagDistance = filterParameters.aprilTagWidth().in(Units.Meters)
      / (2 * Math.tan(filterParameters.estimatedFOV().getRadians() / 2));
    // estimated tag distance in meters
    return (1 / Math.sqrt(averageTagArea / 100)) * minTagDistance;
  }

  private Matrix<N3, N1> calculateStandardDeviations(double tagDistance, boolean multiTagPose) {
    double xyStandardDeviations;
    double thetaStandardDeviations;

    if (multiTagPose) {
      xyStandardDeviations = filterParameters.xyStandardDevBase();
      thetaStandardDeviations = filterParameters.rotStandardDevBase();
    } else {
      // scale the deviations with distance
      xyStandardDeviations = filterParameters.xyStandardDevBase() * Math.pow(tagDistance, 2);
      // don't use rotation with only one tag
      thetaStandardDeviations = 1e4;
    }

    return VecBuilder.fill(xyStandardDeviations, xyStandardDeviations, thetaStandardDeviations);
  }

  // A vision measurement
//  public record VisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs) {
//
//    public VisionMeasurement {}
//  }
}

