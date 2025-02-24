package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.util.FieldRelativeSpeeds;


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

    // Set tolerence in meters
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    thetaController.setTolerance(Units.degreesToRadians(10));
  }

  @Override
  public void initialize() {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    FieldRelativeSpeeds robotSpeeds = RobotState.getInstance().getLastFieldRelativeSpeeds();

    TrapezoidProfile.State xState = new TrapezoidProfile.State(robotPose.getX(), robotSpeeds.vx);
    TrapezoidProfile.State yState = new TrapezoidProfile.State(robotPose.getY(), robotSpeeds.vy);
    TrapezoidProfile.State thetaState = new TrapezoidProfile.State(robotPose.getRotation().getRadians(), robotSpeeds.omega);

    xController.reset(xState);
    yController.reset(yState);
    thetaController.reset(thetaState);
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

  private boolean robotAtPose() {
    return xController.atSetpoint()
        && yController.atSetpoint()
        && thetaController.atSetpoint();
  }
}
