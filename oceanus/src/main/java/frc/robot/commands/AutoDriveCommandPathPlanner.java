package frc.robot.commands;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.List;


public class AutoDriveCommandPathPlanner extends Command {
  private final DriveSubsystem driveSubsystem;

  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;

  private final HolonomicDriveController driveController;

  private Pose2d goal;
  private Trajectory traj;
  private PathPlannerPath path;
  private final Timer timer = new Timer();

  public AutoDriveCommandPathPlanner(DriveSubsystem driveSubsystem, Pose2d goal) {
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
        Units.inchesToMeters(0.25),
        Units.inchesToMeters(0.25),
        Rotation2d.fromDegrees(0.25)
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

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        RobotState.getInstance().getEstimatedPose(),
        new Pose2d(
            goal.getX(),
            goal.getY(),
            RobotState.getInstance().getRotation()
        )
    );

    path = new PathPlannerPath(
        waypoints,
        new PathConstraints(
            Units.feetToMeters(4.0), Units.feetToMeters(4.0),
            2 * Math.PI, 2 * Math.PI
        ),
        null,
        new GoalEndState(0.0, goal.getRotation())
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
