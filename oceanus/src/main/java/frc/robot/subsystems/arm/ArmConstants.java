package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
  public static final int PIVOT_ID = 16;
  public static final int ROLLER_ID = 17;
  public static final int ENCODER_ID = 18;
  public static final int LASER_CAN_ID = 27;

  public static final double PIVOT_GEAR_RATIO = (25.0) * (72.0 / 22.0);
  public static final Rotation2d PIVOT_ENCODER_OFFSET = Rotation2d.fromRotations(0.0);

  public static final double KP = 0.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double KG = 0.0;

  public static final Rotation2d ARM_HOME_SETPOINT = Rotation2d.fromDegrees(-90.0);
  public static final Rotation2d L2_SETPOINT = Rotation2d.fromDegrees(-45.0);
  public static final Rotation2d L3_SETPOINT = Rotation2d.fromDegrees(-45.0);
  public static final Rotation2d L4_SETPOINT = Rotation2d.fromDegrees(45.0);
  public static final Rotation2d BARGE_SETPOINT = Rotation2d.fromDegrees(-90.0);
  public static final Rotation2d ALGAE_SETPOINT = Rotation2d.fromDegrees(-90.0);
  public static final Rotation2d INTAKE_SETPOINT = Rotation2d.fromDegrees(-90.0);
}
