package frc.robot.supersystem;

import frc.robot.RobotState;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import org.dyn4j.geometry.Polygon;

public class Supersystem {
  public enum SupersystemState {
    DISABLED,
    HOME,
    L2,
    L3,
    L4,
    BARGE,
    ALGAE_L2,
    ALGAE_L3,
  }

  private final ElevatorSubsystem elevatorSubsystem;
  private final ArmSubsystem armSubsystem;

  private SupersystemState desiredState = SupersystemState.DISABLED;
  private SupersystemState currentState = SupersystemState.DISABLED;

  public Supersystem(
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem
  ) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
  }

  public void periodic() {
    // if we are coming or leaving home, go to a clearance state
    if (
        (desiredState == SupersystemState.HOME && !elevatorSubsystem.atHome())
        || (desiredState != SupersystemState.HOME && elevatorSubsystem.atHome())
    ) {
      elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.HOME_CLEAR_SETPOINT::getValue);
      armSubsystem.setArmPosition(ArmConstants.ARM_HOME_SETPOINT);
      // wait until everything is clear
      return;
    } else if (desiredState == SupersystemState.L4 && RobotState.getInstance().inReefZone()) {
      // we're going into L4, and we're in the reef location
      elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.L4_SETPOINT::getValue);
      armSubsystem.setArmPosition(ArmConstants.L4_SETPOINT);
      return;
    }

    // Everything else passes, so we can move the arm and elevator to the desired locations
    switch (desiredState) {
      case DISABLED -> {
        elevatorSubsystem.setDisabled();
      }
      case HOME -> {
        elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.HOME_SETPOINT::getValue);
        armSubsystem.setArmPosition(ArmConstants.ARM_HOME_SETPOINT);
      }
      case L2 -> {
        elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.L2_SETPOINT::getValue);
        armSubsystem.setArmPosition(ArmConstants.L2_SETPOINT);
      }
      case L3 -> {
        elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.L3_SETPOINT::getValue);
        armSubsystem.setArmPosition(ArmConstants.L3_SETPOINT);
      }
      case L4 -> {
        elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.L4_SETPOINT::getValue);
        armSubsystem.setArmPosition(ArmConstants.L4_SETPOINT);
      }
      case BARGE -> {
        elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.BARGE_SETPOINT::getValue);
        armSubsystem.setArmPosition(ArmConstants.BARGE_SETPOINT);
      }
      case ALGAE_L2 -> {
        elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.ALGAE_L2_SETPOINT::getValue);
        armSubsystem.setArmPosition(ArmConstants.ALGAE_SETPOINT);
      }
      case ALGAE_L3 -> {
        elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.ALGAE_L3_SETPOINT::getValue);
        armSubsystem.setArmPosition(ArmConstants.ALGAE_SETPOINT);
      }
    }
  }
}
