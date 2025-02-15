package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralscoral.CoralScoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.ChoreoUtils;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.List;

public class AutoCommands {
  public enum StartingPositions {
    StartingPoseLeft,
    StartingPoseRight,
    StartingPoseCenter
  }

  public enum CoralLocations {
    A,
    B,
    AB,
    C,
    D,
    CD,
    E,
    F,
    EF,
    G,
    H,
    GH,
    I,
    J,
    IJ,
    K,
    L,
    KL
  }

  public enum HumanPlayerSide {
    HumanPlayerRightFar,
    HumanPlayerRightClose,
    HumanPlayerLeftFar,
    HumanPlayerLeftClose
  }

  public static Command followChoreoPath(String name) {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(name));
    } catch (IOException | ParseException e) {
      throw new RuntimeException(e);
    }
  }

  public static Command resetPoseAndFollowChoreoPath(DriveSubsystem drive, String name) {
    return Commands.sequence(
        drive.resetPoseFactory(ChoreoUtils.getPathStartingPose(name).getPose()),
        followChoreoPath(name)
    );
  }

  public static Command choreoSequence(DriveSubsystem drive,
                                       CoralScoralSubsystem coral,
                                       StartingPositions start,
                                       HumanPlayerSide humanPlayerSide,
                                       List<CoralLocations> coralPoses) {

    

    return Commands.none();
  }
}
