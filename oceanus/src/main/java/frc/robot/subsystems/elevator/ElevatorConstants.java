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

  public static final double ELEVATOR_KP = 50.0;
  public static final double ELEVATOR_KI = 0.0;
  public static final double ELEVATOR_KD = 0.0;
  public static final double ELEVATOR_KS = 0.1;
  public static final double ELEVATOR_KG = 0.35;

  public static final GosDoubleProperty HOME_SETPOINT  =
      new GosDoubleProperty(false, "ElevatorSetpoints/Home Setpoint", 0.0);
  public static final GosDoubleProperty L1_SETPOINT  =
      new GosDoubleProperty(false, "ElevatorSetpoints/L1 Setpoint", 0.0);
  public static final GosDoubleProperty L2_SETPOINT  =
      new GosDoubleProperty(false, "ElevatorSetpoints/L2 Setpoint", 0.0);
  public static final GosDoubleProperty L3_SETPOINT  =
      new GosDoubleProperty(false, "ElevatorSetpoints/L3 Setpoint", 0.0);
  public static final double L4_SETPOINT = 30.75;
  public static final double BARGE_SETPOINT = 32.0;
  public static final GosDoubleProperty HOME_CLEAR_SETPOINT =
      new GosDoubleProperty(false, "ElevatorSetpoints/Home Clearance Setpoint", 0.0);
  public static final GosDoubleProperty ALGAE_L2_SETPOINT =
      new GosDoubleProperty(false, "ElevatorSetpoints/Algae L2", 0.0);
  public static final GosDoubleProperty ALGAE_L3_SETPOINT =
      new GosDoubleProperty(false, "ElevatorSetpoints/Algae L3", 0.0);

  public static double INTAKE_SETPOINT = 0.0;
}
