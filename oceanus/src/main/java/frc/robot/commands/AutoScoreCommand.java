package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;


public class AutoScoreCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  public AutoScoreCommand(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.driveSubsystem, this.elevatorSubsystem);

    xController = new ProfiledPIDController(
        DriveConstants.XY_KP, 0.0, 0.0,
        DriveConstants.XY_CONSTRAINTS
    );
    yController = new ProfiledPIDController(
        DriveConstants.XY_KP, 0.0, 0.0,
        DriveConstants.XY_CONSTRAINTS
    );
    thetaController = new ProfiledPIDController(
        DriveConstants.THETA_KP, 0.0, 0.0,
        DriveConstants.THETA_CONSTRAINTS
    );
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {

  }
}
