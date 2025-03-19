package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.util.FieldRelativeSpeeds;
import org.littletonrobotics.junction.Logger;

import java.util.List;


public class AutoDriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;

  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;

  private final HolonomicDriveController driveController;

  private Pose2d goal;
  private Trajectory traj;
  private final Timer timer = new Timer();

  public AutoDriveCommand(DriveSubsystem driveSubsystem, Pose2d goal) {
    this.driveSubsystem = driveSubsystem;
    this.goal = goal;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.driveSubsystem);

    xController = new PIDController(
        DriveConstants.XY_KP, 0.0, 0.0
    );
    yController = new PIDController(
        DriveConstants.XY_KP, 0.0, 0.0
    );
    thetaController = new ProfiledPIDController(
        DriveConstants.THETA_KP, 0.0, 0.0,
        DriveConstants.THETA_CONSTRAINTS
    );

    driveController = new HolonomicDriveController(
        xController,
        yController,
        thetaController
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

    // Set tolerence in meters
    driveController.setTolerance(new Pose2d(
        Units.inchesToMeters(0.5),
        Units.inchesToMeters(0.5),
        Rotation2d.fromDegrees(0.5)
    ));
  }

  @Override
  public void initialize() {
    traj = TrajectoryGenerator.generateTrajectory(
        RobotState.getInstance().getEstimatedPose(),
        List.of(),
        goal,
        DriveConstants.TRAJECTORY_CONFIG
    );

    timer.restart();
  }

  @Override
  public void execute() {
    var state = traj.sample(timer.get());
    ChassisSpeeds speeds = driveController.calculate(
        RobotState.getInstance().getEstimatedPose(),
        state,
        goal.getRotation()
    );

    Logger.recordOutput("AutoAlign/X output", speeds.vxMetersPerSecond);
    Logger.recordOutput("AutoAlign/Y output", speeds.vyMetersPerSecond);
    Logger.recordOutput("AutoAlign/Theta output", speeds.omegaRadiansPerSecond);

    Logger.recordOutput("AutoAlign/End Goal Pose", goal);

    Logger.recordOutput("AutoAlign/Setpoint Pose", state.poseMeters);

    driveSubsystem.runVelocity(speeds);
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
    return traj.getTotalTimeSeconds() < timer.get();
  }
}
