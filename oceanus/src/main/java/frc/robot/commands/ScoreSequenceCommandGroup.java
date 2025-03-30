package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.swerve.SwerveDrivePIDToPose;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.supersystem.Supersystem;

import java.util.Objects;

public class ScoreSequenceCommandGroup extends SequentialCommandGroup {
  public ScoreSequenceCommandGroup(
      DriveSubsystem swerve,
      Supersystem supersystem,
      boolean left) {
    Supersystem.SupersystemState goalState = Supersystem.SupersystemState.L2;
    RobotState.CoralLevel coralLevel = RobotState.getInstance().getCoralLevel();
    if (Objects.requireNonNull(coralLevel) == RobotState.CoralLevel.L3) {
      goalState = Supersystem.SupersystemState.L3;
    } else if (coralLevel == RobotState.CoralLevel.L4) {
      goalState = Supersystem.SupersystemState.L4;
    }

    Supersystem.SupersystemState finalGoalState = goalState;
    addCommands(
        new WaitUntilCommand(() -> isClear(supersystem, finalGoalState))
            .deadlineFor(new SwerveDrivePIDToPose(swerve, swerve::getClosestClearance))
            .alongWith(supersystem.setDesiredState(finalGoalState)),
        new SwerveDrivePIDToPose(swerve, () -> swerve.getClosestBranch(left))
    );
  }

  private boolean isClear(Supersystem supersystem, Supersystem.SupersystemState state) {
    return supersystem.atSetpoint() && supersystem.getDesiredState() == state;
  }
}