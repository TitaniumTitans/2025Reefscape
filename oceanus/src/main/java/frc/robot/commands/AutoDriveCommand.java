package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.util.FieldRelativeSpeeds;
import org.littletonrobotics.junction.Logger;


public class AutoDriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private Pose2d goal;

  public AutoDriveCommand(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, Pose2d goal) {
    this.driveSubsystem = driveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.goal = goal;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.driveSubsystem);

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

//    xController = new PIDController(
//        DriveConstants.XY_KP, 0.0, 0.0
//    );
//    yController = new PIDController(
//        DriveConstants.XY_KP, 0.0, 0.0
//    );
//    thetaController = new PIDController(
//        DriveConstants.THETA_KP, 0.0, 0.0
//    );

    thetaController.enableContinuousInput(Units.degreesToRadians(-180), Units.degreesToRadians(180));

    // Set tolerence in meters
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    thetaController.setTolerance(Units.degreesToRadians(10));
  }

  @Override
  public void initialize() {
//    goal = RobotState.getInstance().getEstimatedPose().plus(
//        new Transform2d(1.0, 0.0, new Rotation2d())
//    );

    xController.setGoal(goal.getX());
    yController.setGoal(goal.getY());
    thetaController.setGoal(goal.getRotation().getRadians());

    xController.reset(RobotState.getInstance().getEstimatedPose().getX());
    yController.reset(RobotState.getInstance().getEstimatedPose().getY());
    thetaController.reset(RobotState.getInstance().getEstimatedPose().getRotation().getRadians());
  }

  @Override
  public void execute() {
    double x = xController.calculate(RobotState.getInstance().getEstimatedPose().getX());
    double y = yController.calculate(RobotState.getInstance().getEstimatedPose().getY());
    double theta = thetaController.calculate(
        RobotState.getInstance().getEstimatedPose().getRotation().getRadians());

    Logger.recordOutput("AutoAlign/X output", x);
    Logger.recordOutput("AutoAlign/Y output", y);
    Logger.recordOutput("AutoAlign/Theta output", theta);

    Logger.recordOutput("AutoAlign/End Goal Pose", goal);

    Logger.recordOutput("AutoAlign/Setpoint Pose", new Pose2d(
        xController.getSetpoint().position,
        yController.getSetpoint().position,
        Rotation2d.fromRadians(thetaController.getSetpoint().position)
    ));

    ChassisSpeeds speeds = new ChassisSpeeds(
        x,
        y,
        theta
    );

    boolean isFlipped = false;
//        DriverStation.getAlliance().isPresent()
//            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    Rotation2d rotation = RobotState.getInstance().getRotation();
    driveSubsystem.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? rotation.plus(new Rotation2d(Math.PI)) : rotation));
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return robotAtPose();
  }

  @Override
  public void end(boolean interrupted) {

  }

  private boolean robotAtPose() {
    return xController.atGoal()
        && yController.atGoal()
        && thetaController.atGoal();
  }
}
