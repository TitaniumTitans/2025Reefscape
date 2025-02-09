package frc.robot.commands;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.coralscoral.CoralScoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoSelector {
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  public AutoSelector(DriveSubsystem drive, CoralScoralSubsystem coral) {
    NamedCommands.registerCommand("score", coral.setScorerPowerFactory(2.5).withTimeout(0.5));
    NamedCommands.registerCommand("print", Commands.print("Waiting"));

    autoChooser.addDefaultOption("3L1Left", AutoCommands.resetPoseAndFollowChoreoPath(drive, "3L1Left"));
//    autoChooser.addOption("L1HP", AutoCommands.resetPoseAndFollowChoreoPath(drive, "L1HP"));
    autoChooser.addOption("1L1Right", AutoCommands.resetPoseAndFollowChoreoPath(drive, "1L1Right"));
    autoChooser.addOption("2L1Right", AutoBuilder.buildAuto("2L1Right"));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutoCommand() {
    return autoChooser.get();
  }
}
