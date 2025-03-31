package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ChoreoPoses;
import frc.robot.commands.swerve.SwerveDrivePIDToPose;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.supersystem.Supersystem;
import frc.robot.util.ChoreoUtils;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoSelector {
  private final LoggedDashboardChooser<Command> chooser =
      new LoggedDashboardChooser<>("AutoChooser");

  public AutoSelector(DriveSubsystem drive, Supersystem supersystem, CoralSubsystem coral) {
    NamedCommands.registerCommand("ArmUp", supersystem.setDesiredState(Supersystem.SupersystemState.L4));

    chooser.addDefaultOption("None", Commands.none());
    // One piece
    chooser.addOption("1L4LeftJ",
        AutoCommands.resetPoseAndFollowChoreoPath(drive, "LeftJ")
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(1.0))
            .andThen(supersystem.runArmRollers(0.0))
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("JHPLeft")));

    chooser.addOption("1L4LeftJ",
        AutoCommands.resetPoseAndFollowChoreoPath(drive, "LeftJ")
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(1.0))
            .andThen(supersystem.runArmRollers(0.0))
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("JHPLeft")));

    chooser.addOption("1L4CenterG",
        AutoCommands.resetPoseAndFollowChoreoPath(drive, "CenterG")
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(1.0))
            .andThen(supersystem.runArmRollers(0.0))
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("GCenter")));
    chooser.addOption("1L4CenterH",
        AutoCommands.resetPoseAndFollowChoreoPath(drive, "CenterH")
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(1.0))
            .andThen(supersystem.runArmRollers(0.0))
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("HCenter")));

    // Two piece
    chooser.addOption("2L4LeftJK",
        AutoCommands.resetPoseAndFollowChoreoPath(drive, "LeftJ")
            // score
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(supersystem.runArmRollers(0.0))
            // HP
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("JHPLeft"))
            .andThen(AutoCommands.intakeUntilCoral(coral, supersystem))
            .andThen(AutoCommands.intakeStopCommand(coral, supersystem))
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.L4))
            .andThen(AutoCommands.followChoreoPath("HPLeftK"))
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(supersystem.runArmRollers(0.0))
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("KHPLeft"))
    );

    chooser.addOption("2L4RightED",
        AutoCommands.resetPoseAndFollowChoreoPath(drive, "RightE")
            // score
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(supersystem.runArmRollers(0.0))
            // HP
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("EHPRight"))
            .andThen(AutoCommands.intakeUntilCoral(coral, supersystem))
            .andThen(AutoCommands.intakeStopCommand(coral, supersystem))
            // score 2
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.L4))
            .andThen(AutoCommands.followChoreoPath("HPRightD"))
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(supersystem.runArmRollers(0.0))
            // hp
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("DHPRight"))
    );

    // Three pieces
    chooser.addOption("3L4RightEDC",
        AutoCommands.resetPoseAndFollowChoreoPath(drive, "RightE")
            // score
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(supersystem.runArmRollers(0.0))
            // HP
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("EHPRight"))
            .andThen(AutoCommands.intakeUntilCoral(coral, supersystem))
            .andThen(AutoCommands.intakeStopCommand(coral, supersystem))
            // score 2
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.L4))
            .andThen(AutoCommands.followChoreoPath("HPRightD"))
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(supersystem.runArmRollers(0.0))
            // hp
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("DHPRight"))
            .andThen(AutoCommands.intakeUntilCoral(coral, supersystem))
            .andThen(AutoCommands.intakeStopCommand(coral, supersystem))
            // score C
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.L4))
            .andThen(AutoCommands.followChoreoPath("HPRightC"))
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(supersystem.runArmRollers(0.0))
            // hp
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("CHPRight"))
            .andThen(AutoCommands.intakeUntilCoral(coral, supersystem))
            .andThen(AutoCommands.intakeStopCommand(coral, supersystem))
    );

    chooser.addOption("3L4LeftJKL",
        AutoCommands.resetPoseAndFollowChoreoPath(drive, "LeftJ")
            // score
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(supersystem.runArmRollers(0.0))
            // HP
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("JHPLeft"))
            .andThen(AutoCommands.intakeUntilCoral(coral, supersystem))
            .andThen(AutoCommands.intakeStopCommand(coral, supersystem))
            // score
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.L4))
            .andThen(AutoCommands.followChoreoPath("HPLeftK"))
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(supersystem.runArmRollers(0.0))
            // hp
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("KHPLeft"))
            .andThen(AutoCommands.intakeUntilCoral(coral, supersystem))
            .andThen(AutoCommands.intakeStopCommand(coral, supersystem))
            // score
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.L4))
            .andThen(AutoCommands.followChoreoPath("HPLeftL"))
            .andThen(supersystem.runArmRollers(3.0))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(supersystem.runArmRollers(0.0))
            // hp
            .andThen(supersystem.setDesiredState(Supersystem.SupersystemState.HOME))
            .andThen(AutoCommands.followChoreoPath("LHPLeft"))
            .andThen(AutoCommands.intakeUntilCoral(coral, supersystem))
            .andThen(AutoCommands.intakeStopCommand(coral, supersystem))
    );
  }

  public Command getAutoCommand() {
    return chooser.get();
  }
}
