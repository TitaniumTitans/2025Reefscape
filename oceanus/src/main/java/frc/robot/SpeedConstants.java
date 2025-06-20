package frc.robot;

import edu.wpi.first.math.util.Units;

public class SpeedConstants {
  public static final double DRIVETRAIN_SPEED_MPS = Units.feetToMeters(17.1); // 17.1
  public static final double DRIVETRAIN_ANGULAR_SPEED_PERCENTAGE = 1.0; // 1.0
  public static final double ARM_MAX_SPEED_DEGREES = 370; // 370
  public static final double ARM_MAX_ACCELERATION_DEGREES = 700; // 700
  public static final double ELEVATOR_MAX_SPEED_MPS = 2; // 2
  public static final double ELEVATOR_MAX_ACCELERATION_MPS = 5; // 5
  public static final double MAX_ALIGNMENT_LINEAR_VELOCITY = 4; //4
  public static final double MAX_ALIGNMENT_LINEAR_ACCELERATION = 15; //15
  public static final Number MAX_ALIGNMENT_ANGULAR_VELOCITY = Units.degreesToRadians(400.0); // rads/sec
  public static final Number MAX_ALIGNMENT_ANGULAR_ACCELERATION = Units.degreesToRadians(900.0); // rads/sec/sec
}
