package frc.robot.commands;

import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoSelector {
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  public AutoSelector() {
    new EventTrigger("intake").onTrue(Commands.print("Intaking!!!"));
  }

  public Command getAutoCommand() {
    return autoChooser.get();
  }
}
