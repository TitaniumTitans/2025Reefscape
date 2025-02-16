package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class MechanismVisualizer {


  private final String key = "Visualizer";

  @Setter
  private double intakeAngleDegrees;
  @Setter
  private double coralAngleDegrees;
  @Setter
  private double elevatorHeightInches;

  private static MechanismVisualizer instance;

  public static MechanismVisualizer getInstance() {
    if (instance == null) instance = new MechanismVisualizer();
    return instance;
  }

  private MechanismVisualizer() {
    intakeAngleDegrees = 0.0;
    coralAngleDegrees = 0.0;
    elevatorHeightInches = 0.0;
  }

  public void updateVisualization() {
    Pose3d intakePose = new Pose3d();
    Pose3d coralPose = new Pose3d();

    Pose3d lowerStagePose = new Pose3d(
      0.0, 0.0, Units.inchesToMeters(elevatorHeightInches) / 2.0,
      new Rotation3d()
    );

    Pose3d upperStagePose = new Pose3d(
        0.0, 0.0,
        Units.inchesToMeters(elevatorHeightInches) * ((70.0 - 6.0) / 70.0),
        new Rotation3d()
    );

    Logger.recordOutput(key + "/3D Pose",
        lowerStagePose, upperStagePose);
  }
}
