package frc.robot.supersystem;

import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class Supersystem {
  private final ElevatorSubsystem elevatorSubsystem;

  public Supersystem(
      ElevatorSubsystem elevatorSubsystem
  ) {
    this.elevatorSubsystem = elevatorSubsystem;
  }
}
