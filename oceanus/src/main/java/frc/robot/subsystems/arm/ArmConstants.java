package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
  public static int PIVOT_ID = 16;

  public static double PIVOT_GEAR_RATIO = (1.0 / 25.0) * (72.0 / 22.0);
  public static Rotation2d PIVOT_ENCODER_OFFSET = Rotation2d.fromRotations(0.0);
}
