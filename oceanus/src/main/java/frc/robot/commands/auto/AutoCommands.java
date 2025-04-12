package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.supersystem.Supersystem;
import frc.robot.util.ChoreoUtils;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.List;

public class AutoCommands {
  public enum StartingPositions {
    StartingPosLeft,
    StartingPosRight,
    StartingPosCenter
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
        drive.resetPoseFactory(ChoreoUtils.getPathStartingPose(name)::getPose),
        followChoreoPath(name)
    );
  }

  public static Command dropCoral(Supersystem supersystem) {
    return supersystem.runArmRollers(5.0)
        .andThen(Commands.waitSeconds(0.25))
        .andThen(supersystem.runArmRollers(0.0));
  }

  public static Command intakeUntilCoral(CoralSubsystem coral, Supersystem supersystem) {
    return supersystem.setDesiredState(Supersystem.SupersystemState.INTAKE)
        .andThen(supersystem.runArmRollers(-3.5) // -2.25
            .alongWith(coral.setScoringVoltages(6.0, 3.0, 3.0)))// 3.5
        .repeatedly()
        .until(supersystem::hasCoral);
  }

  public static Command intakeStopCommand(CoralSubsystem coral, Supersystem supersystem) {
    return supersystem.setDesiredState(Supersystem.SupersystemState.HOME)
        .andThen(supersystem.runArmRollers(0.0))
        .andThen(coral.setScoringVoltages(0.0, 0.0, 0.0));
  }
}
