package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double ODOMETRY_FREQUENCY = 250;
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(20.75);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(20.75);
  public static final Translation2d[] MODULE_TRANSLATIONS = {
      new Translation2d(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
      new Translation2d(TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2),
      new Translation2d(-TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
      new Translation2d(-TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2)
  };

  public static final double MAX_LINEAR_SPEED_MPS = Units.feetToMeters(16.0);

  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
}
