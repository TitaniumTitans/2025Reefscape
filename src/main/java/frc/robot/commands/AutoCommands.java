package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.ChoreoUtils;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class AutoCommands {
  public Command followChoreoPath(String name) {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(name));
    } catch (IOException | ParseException e) {
      throw new RuntimeException(e);
    }
  }

  public Command resetPoseAndFollowChoreoPath(DriveSubsystem drive, String name) {
    return Commands.sequence(
        drive.resetPose(ChoreoUtils.getPathStartingPose(name).getPose()),
        followChoreoPath(name)
    );
  }
}
