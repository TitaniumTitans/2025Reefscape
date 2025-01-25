package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import org.photonvision.simulation.SimCameraProperties;

public class VisionConstants {
  public static final VisionFilterParameters FILTER_PARAMETERS = new VisionFilterParameters(
      0.5,
      1.5,
      Units.Centimeters.of(21),
      0.5,
      Rotation2d.fromDegrees(70),
      Units.Centimeters.of(805),
      Units.Centimeters.of(1755),
      Units.Centimeters.of(15));

  public static final SimCameraProperties SIM_CAMERA_PROPERTIES = new SimCameraProperties();

  static {
    SIM_CAMERA_PROPERTIES.setCalibration(800, 600, Rotation2d.fromDegrees(100));
    SIM_CAMERA_PROPERTIES.setCalibError(0.25, 0.08);
    SIM_CAMERA_PROPERTIES.setFPS(40);
    SIM_CAMERA_PROPERTIES.setAvgLatencyMs(20);
    SIM_CAMERA_PROPERTIES.setLatencyStdDevMs(10);
  }

  public static final Transform3d SIM_CAMERA_TRANSFORM = new Transform3d();
}
