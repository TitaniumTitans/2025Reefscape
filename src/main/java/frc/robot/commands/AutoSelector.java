package frc.robot.commands;

import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoSelector {
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  public AutoSelector(DriveSubsystem drive) {
    new EventTrigger("intake").onTrue(Commands.print("Intaking!!!"));

    autoChooser.addDefaultOption("3L1Left", AutoCommands.resetPoseAndFollowChoreoPath(drive, "3L1Left"));
  }

  public Command getAutoCommand() {
    return autoChooser.get();
  }
}
