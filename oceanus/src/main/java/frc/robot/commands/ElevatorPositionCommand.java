package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;


public class ElevatorPositionCommand {
  private final AlgaeSubsystem algaeSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  public ElevatorPositionCommand(AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem) {
    super();
    this.algaeSubsystem = algaeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
  }
  
}
