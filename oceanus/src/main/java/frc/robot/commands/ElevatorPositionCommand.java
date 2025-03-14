package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import java.util.Set;


public class ElevatorPositionCommand {
  public enum ScoringPose {
    HOME, L2, L3, L4, BARGE
  }
  private final AlgaeSubsystem algaeSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final ArmSubsystem armSubsystem;

  public ElevatorPositionCommand(
      AlgaeSubsystem algaeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
  }

  public Command getCommand(ScoringPose setpoint) {
    return Commands.defer(
        () -> {
          SequentialCommandGroup group = new SequentialCommandGroup();

          // check if going down to home, get the arm into home position
          // if going up from home we also need to make sure that the arm is in the home state
          if ((setpoint == ScoringPose.HOME && !elevatorSubsystem.underClearance())
              || (setpoint != ScoringPose.HOME && elevatorSubsystem.underClearance())) {
            group.addCommands(armSubsystem.setArmPositionFactory(Rotation2d.kCW_90deg));
            group.addCommands(armSubsystem.waitUntilAtSetpoint());
            group.addCommands(elevatorSubsystem.setElevatorSetpointFactory(ElevatorConstants.HOME_CLEAR_SETPOINT::getValue));
          }

          switch (setpoint) {
            case HOME -> {
              group.addCommands(armSubsystem.setArmPositionFactory(Rotation2d.kCW_90deg));
              group.addCommands(armSubsystem.waitUntilAtSetpoint());
              group.addCommands(
                  elevatorSubsystem.setElevatorSetpointFactory(
                      ElevatorConstants.HOME_SETPOINT::getValue
                  ));
            }
            case L2 -> {
              group.addCommands(
                  elevatorSubsystem.setElevatorSetpointFactory(
                      ElevatorConstants.L2_SETPOINT::getValue
                  ));
            }
            case L3 -> {
              group.addCommands(
                  elevatorSubsystem.setElevatorSetpointFactory(
                      ElevatorConstants.L3_SETPOINT::getValue
                  ));
            }
            case L4 -> {
              group.addCommands(
                  elevatorSubsystem.setElevatorSetpointFactory(
                      ElevatorConstants.L4_SETPOINT::getValue
                  ));
            }
            case BARGE -> {
              group.addCommands(
                  elevatorSubsystem.setElevatorSetpointFactory(
                      ElevatorConstants.BARGE_SETPOINT::getValue
                  ));
            }
          }

          group.addRequirements(elevatorSubsystem, algaeSubsystem, armSubsystem);
          return group;
        },
        Set.of(elevatorSubsystem, algaeSubsystem, armSubsystem)
    );
  }
}
