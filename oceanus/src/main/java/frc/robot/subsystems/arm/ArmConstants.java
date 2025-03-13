package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
  public static final int PIVOT_ID = 16;
  public static final int ROLLER_ID = 17;
  public static final int ENCODER_ID = 18;

  public static final double PIVOT_GEAR_RATIO = (25.0) * (72.0 / 22.0);
  public static final Rotation2d PIVOT_ENCODER_OFFSET = Rotation2d.fromRotations(0.0);

  public static final double KP = 0.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double KG = 0.0;
}
