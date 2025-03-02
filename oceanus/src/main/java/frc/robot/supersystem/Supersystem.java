package frc.robot.supersystem;

import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class Supersystem {
  public enum SupersystemState {
    DISABLED,
    HOME,
    LEAVING_HOME,
    COMING_HOME,
    L1,
    L2,
    L3,
    L4,
    BARGE,
    PROCESSOR,
    ALGAE_L2,
    ALGAE_L3,
    GROUND_ALGAE,
    GROUND_CORAL,
    HP_CORAL,
  }

  private final ElevatorSubsystem elevatorSubsystem;
  private final CoralSubsystem coralSubsystem;

  private SupersystemState desiredState = SupersystemState.DISABLED;

  public Supersystem(
      ElevatorSubsystem elevatorSubsystem,
      CoralSubsystem coralSubsystem
  ) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralSubsystem = coralSubsystem;
  }

  public void periodic() {
    // handle the desired state
    switch (desiredState) {
      case DISABLED -> {
      }
      case HOME -> {
        coralSubsystem.setPivotAngle(0.0);
        elevatorSubsystem.setElevatorSetpoint(() -> 0.0);
      }
      case LEAVING_HOME -> {
      }
      case COMING_HOME -> {
      }
      case L1 -> {
      }
      case L2 -> {
      }
      case L3 -> {
      }
      case L4 -> {
      }
      case BARGE -> {
      }
      case PROCESSOR -> {
      }
      case ALGAE_L2 -> {
      }
      case ALGAE_L3 -> {
      }
      case GROUND_ALGAE -> {
      }
      case GROUND_CORAL -> {
      }
      case HP_CORAL -> {
      }
    }
  }
}
