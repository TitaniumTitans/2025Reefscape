package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoSelector {
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  public AutoSelector(DriveSubsystem drive) {
    new EventTrigger("intake").onTrue(Commands.print("Intaking!!!"));
    NamedCommands.registerCommand("intake", Commands.print("Intaking!!"));

    autoChooser.addDefaultOption("3L1Left", AutoCommands.resetPoseAndFollowChoreoPath(drive, "3L1Left"));
    autoChooser.addOption("1L1Right", AutoCommands.resetPoseAndFollowChoreoPath(drive, "1L1Right"));
    autoChooser.addOption("L1HP", AutoCommands.resetPoseAndFollowChoreoPath(drive, "L1HP"));

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
