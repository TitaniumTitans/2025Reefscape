package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import java.util.Set;


public class ElevatorPositionCommand {
  public enum ScoringPose {
    HOME, L2, L3, L4, BARGE
  }
  private final AlgaeSubsystem algaeSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  public ElevatorPositionCommand(AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
  }

  public Command getCommand(ScoringPose height) {
    return Commands.defer(
        () -> {
          SequentialCommandGroup group = new SequentialCommandGroup();

          if (height == ScoringPose.HOME )

          group.addRequirements(elevatorSubsystem, algaeSubsystem);
          return group;
        },
        Set.of(elevatorSubsystem, algaeSubsystem)
    );
  }
}
