package frc.robot.subsystems.elevator;

import com.gos.lib.properties.GosDoubleProperty;

public class ElevatorConstants {
  public static final int MASTER_MOTOR_ID = 14;
  public static final int FOLLOWER_MOTOR_ID = 15;

  public static final double GEAR_REDUCTION = (12.0 / 36.0);
  // N = number of pulley grooves
  // P = pulley groove pitch, 5mm
  // pd = PN/pi
  public static final double SPOOL_DIAMETER_METERS = ((5.0 * 24.0) / Math.PI) / 1000;

  public static final double ELEVATOR_KP = 0.0;
  public static final double ELEVATOR_KI = 0.0;
  public static final double ELEVATOR_KD = 0.0;

  public static final GosDoubleProperty HOME_SETPOINT  =
      new GosDoubleProperty(false, "ElevatorSetpoints/Home Setpoint", 0.0);
  public static final GosDoubleProperty L1_SETPOINT  =
      new GosDoubleProperty(false, "ElevatorSetpoints/L1 Setpoint", 0.0);
  public static final GosDoubleProperty L2_SETPOINT  =
      new GosDoubleProperty(false, "ElevatorSetpoints/L2 Setpoint", 0.0);
  public static final GosDoubleProperty L3_SETPOINT  =
      new GosDoubleProperty(false, "ElevatorSetpoints/L3 Setpoint", 0.0);
  public static final GosDoubleProperty L4_SETPOINT  =
      new GosDoubleProperty(false, "ElevatorSetpoints/L4 Setpoint", 0.0);
  public static final GosDoubleProperty BARGE_SETPOINT =
      new GosDoubleProperty(false, "ElevatorSetpoints/Barge Setpoint", 0.0);
  public static final GosDoubleProperty HOME_CLEAR_SETPOINT =
      new GosDoubleProperty(false, "ElevatorSetpoints/Home Clearance Setpoint", 0.0);
}
