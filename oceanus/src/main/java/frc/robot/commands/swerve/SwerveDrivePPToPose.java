package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;


public class SwerveDrivePPToPose {
  private final DriveSubsystem driveSubsystem;
  private final Supplier<Pose2d> poseSupplier;

  private PathPlannerPath path;

  public SwerveDrivePPToPose(DriveSubsystem driveSubsystem, Pose2d pose2d) {
    this(driveSubsystem, () -> pose2d);
  }

  public SwerveDrivePPToPose(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.poseSupplier = poseSupplier;
  }

  public Command getDriveCommand() {
    return Commands.defer( () -> {
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(RobotState.getInstance().getEstimatedPose().getTranslation(), pointAt()),
          poseSupplier.get()
      );
      path = new PathPlannerPath(
          waypoints,
          DriveConstants.CONSTRAINTS,
          new IdealStartingState(
              Math.hypot(driveSubsystem.getFieldRelativeSpeeds().vxMetersPerSecond,
                  driveSubsystem.getFieldRelativeSpeeds().vyMetersPerSecond),
              RobotState.getInstance().getRotation()),
          new GoalEndState(0.0, poseSupplier.get().getRotation())
      );

      return AutoBuilder.followPath(path);
    }, Set.of(driveSubsystem));
  }

  public Rotation2d pointAt() {
    return pointAt(RobotState.getInstance().getEstimatedPose(), poseSupplier.get());
  }

  public Rotation2d pointAt(Pose2d start, Pose2d end) {
    return Rotation2d.fromRadians(Math.atan2(end.getY() - start.getY(), end.getX() - start.getX()));
  }
}
