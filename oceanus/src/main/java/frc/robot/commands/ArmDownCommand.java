package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;


public class ArmDownCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private double elevatorSetpointInches = 0.0;
  private Rotation2d armSetpoint = new Rotation2d();

  public ArmDownCommand(ArmSubsystem armSubsystem,
                        ElevatorSubsystem elevatorSubsystem,
                        double elevatorSetpointInches,
                        Rotation2d armSetpoint) {
    this.armSubsystem = armSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorSetpointInches = elevatorSetpointInches;
    this.armSetpoint = armSetpoint;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.armSubsystem, this.elevatorSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    armSubsystem.setArmPosition(armSetpoint);

    if (elevatorSubsystem.atSetpoint(elevatorSetpointInches)) {
      elevatorSubsystem.setElevatorSetpoint(() -> elevatorSetpointInches);
    } else {
      elevatorSubsystem.setDisabled();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setDisabled();
    armSubsystem.setDisabled();
  }
}
