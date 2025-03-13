package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class MechanismVisualizer {


  private final String key = "Visualizer";

  @Setter
  private double algaeAngleDegrees;
  @Setter
  private double coralAngleDegrees;
  @Setter
  private double elevatorHeightInches;
  @Setter
  private double armAngleDegrees;

  private static MechanismVisualizer instance;

  public static MechanismVisualizer getInstance() {
    if (instance == null) instance = new MechanismVisualizer();
    return instance;
  }

  private MechanismVisualizer() {
    algaeAngleDegrees = 0.0;
    coralAngleDegrees = 0.0;
    elevatorHeightInches = 0.0;
  }

  public void updateVisualization() {

    Pose3d elevatorHeight = new Pose3d(
      0.0, 0.0, Units.inchesToMeters(elevatorHeightInches),
      new Rotation3d()
    );

    Pose3d armPose = new Pose3d(
        0.0, 0.0,
        Units.inchesToMeters(elevatorHeightInches) + 0.925, // offset to account for origin
        new Rotation3d(0.0, Units.degreesToRadians((120 - 90) - armAngleDegrees), 0.0)
    );

    Pose3d coralPose = new Pose3d(
        0.0, -0.3, 0.2,
        new Rotation3d()
    );

    Pose3d algaePose = new Pose3d(
        0.325, 0.0, 0.225,
        new Rotation3d(
            0.0,
            Units.degreesToRadians(90 - 65 - algaeAngleDegrees),
            0.0
        )
    );

    Logger.recordOutput(key + "/3D Pose",
        elevatorHeight, armPose, coralPose, algaePose);
    Logger.recordOutput("Test Point", algaePose);
  }

}
