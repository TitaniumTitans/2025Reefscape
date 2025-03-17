package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.util.FieldRelativeSpeeds;
import org.littletonrobotics.junction.Logger;


public class AutoScoreCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  private final Pose2d goal;

  public AutoScoreCommand(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, Pose2d goal) {
    this.driveSubsystem = driveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.goal = goal;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.driveSubsystem);

//    xController = new ProfiledPIDController(
//        DriveConstants.XY_KP, 0.0, 0.0,
//        DriveConstants.XY_CONSTRAINTS
//    );
//    yController = new ProfiledPIDController(
//        DriveConstants.XY_KP, 0.0, 0.0,
//        DriveConstants.XY_CONSTRAINTS
//    );
//    thetaController = new ProfiledPIDController(
//        DriveConstants.THETA_KP, 0.0, 0.0,
//        DriveConstants.THETA_CONSTRAINTS
//    );

    xController = new PIDController(
        DriveConstants.XY_KP, 0.0, 0.0
    );
    yController = new PIDController(
        DriveConstants.XY_KP, 0.0, 0.0
    );
    thetaController = new PIDController(
        DriveConstants.THETA_KP, 0.0, 0.0
    );

    thetaController.enableContinuousInput(Units.degreesToRadians(-180), Units.degreesToRadians(180));

    // Set tolerence in meters
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    thetaController.setTolerance(Units.degreesToRadians(10));
  }

  @Override
  public void initialize() {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    FieldRelativeSpeeds robotSpeeds = RobotState.getInstance().getLastFieldRelativeSpeeds();

//    TrapezoidProfile.State xState = new TrapezoidProfile.State(robotPose.getX(), robotSpeeds.vx);
//    TrapezoidProfile.State yState = new TrapezoidProfile.State(robotPose.getY(), robotSpeeds.vy);
//    TrapezoidProfile.State thetaState = new TrapezoidProfile.State(robotPose.getRotation().getRadians(), robotSpeeds.omega);

//    xController.reset(xState);
//    yController.reset(yState);
//    thetaController.reset(thetaState);
  }

  @Override
  public void execute() {
    double x = xController.calculate(RobotState.getInstance().getEstimatedPose().getX(), goal.getX());
    double y = yController.calculate(RobotState.getInstance().getEstimatedPose().getY(), goal.getY());
    double theta = thetaController.calculate(
        RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
        goal.getRotation().getRadians());

    Logger.recordOutput("AutoAlign/X output", x * DriveConstants.MAX_LINEAR_SPEED_MPS);
    Logger.recordOutput("AutoAlign/Y output", y * DriveConstants.MAX_LINEAR_SPEED_MPS);
    Logger.recordOutput("AutoAlign/Theta output", theta * DriveConstants.MAX_LINEAR_SPEED_MPS);

    Logger.recordOutput("AutoAlign/Goal Pose", goal);

    driveSubsystem.runVelocity(
        new ChassisSpeeds(
            -x * DriveConstants.MAX_LINEAR_SPEED_MPS,
            -y * DriveConstants.MAX_LINEAR_SPEED_MPS,
            theta * DriveConstants.MAX_ANGULAR_SPEED
        )
    );
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
