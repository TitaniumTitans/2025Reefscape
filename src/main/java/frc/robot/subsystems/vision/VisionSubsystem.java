package frc.robot.subsystems.vision;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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

  public Command processVision(Supplier<Pose2d> poseSupplier, Consumer<VisionMeasurement> measurementConsumer) {
    return run(() -> {
      // update the real robot pose
      actualRobotPose = poseSupplier.get();

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

        // use the closest pose
      }
    });
  }

  // A vision measurement
  public record VisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs) {

    public VisionMeasurement {}
  }
}

