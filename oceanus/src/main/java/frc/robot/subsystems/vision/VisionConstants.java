package frc.robot.subsystems.vision;

import com.gos.lib.properties.TunableTransform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import org.photonvision.simulation.SimCameraProperties;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

public class VisionConstants {
  public static final VisionFilterParameters FILTER_PARAMETERS = new VisionFilterParameters(
      0.085, // 0.5
      1.0, // 1.5
      Units.Centimeters.of(21),
      0.6,
      Rotation2d.fromDegrees(70),
      Units.Centimeters.of(805),
      Units.Centimeters.of(1755),
      Units.Centimeters.of(35)); // 15

  public static final SimCameraProperties SIM_CAMERA_PROPERTIES = new SimCameraProperties();

  static {
    SIM_CAMERA_PROPERTIES.setCalibration(800, 600, Rotation2d.fromDegrees(70));
    SIM_CAMERA_PROPERTIES.setCalibError(0.52, 0.08);
    SIM_CAMERA_PROPERTIES.setFPS(40);
    SIM_CAMERA_PROPERTIES.setAvgLatencyMs(20);
    SIM_CAMERA_PROPERTIES.setLatencyStdDevMs(10);
  }

  public static final TunableTransform3d SIM_CAMERA_TRANSFORM = new TunableTransform3d(
      false,
      "CameraTransforms/BackCamera",
      new Transform3d(
          new Translation3d(
              inchesToMeters(-10.5),
              inchesToMeters(0),
              inchesToMeters(16.25)
          ),
          new Rotation3d(0.0, degreesToRadians(0), degreesToRadians(180)))
  );

  public static final TunableTransform3d RIGHT_CAMERA_TRANSFORM = new TunableTransform3d(
      false,
      "CameraTransforms/BackCamera",
      new Transform3d(
      new Translation3d(
          inchesToMeters(0.0),
          inchesToMeters(0),
          inchesToMeters(0.0)
      ),
      new Rotation3d(0.0, degreesToRadians(0), degreesToRadians(0.0)))
  );

  public static final Transform3d BACK_LEFT_TRANSFORM = new Transform3d(
      new Translation3d(
          inchesToMeters(-9.2198),
          inchesToMeters(10.7368),
          inchesToMeters(8.25)
      ),
//      new Rotation3d(
//          degreesToRadians(76.3672), //90 - 76.3672
//          degreesToRadians(0.0), // 65.9
//          degreesToRadians(180-60.0) //180 + 60
//      )
      new Rotation3d(0.0, degreesToRadians(-28.125), 0.0)
          .rotateBy(new Rotation3d(0.0, 0.0, degreesToRadians(-30.0)))
  );
}
