package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotState;
import frc.robot.commands.swerve.SwerveDrivePIDToPose;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.supersystem.Supersystem;

import java.util.Objects;
import java.util.Set;

import static frc.robot.RobotState.CoralLevel.L4;

public class ScoreSequenceCommandGroup extends SequentialCommandGroup {
  public ScoreSequenceCommandGroup(
      DriveSubsystem swerve,
      Supersystem supersystem,
      boolean left) {

    addCommands(
        new WaitUntilCommand(() -> isClear(supersystem))
            .alongWith(new SwerveDrivePIDToPose(swerve, swerve::getClosestClearance))
            .alongWith(armMoveAutoScoreCommand(supersystem)),
        new SwerveDrivePIDToPose(swerve, () -> swerve.getClosestBranch(left))
    );
  }

  public Command armMoveAutoScoreCommand(Supersystem supersystem) {
    return Commands.defer(() -> {
      Command supersystemCommand = Commands.none();
      switch (RobotState.getInstance().getCoralLevel()) {
        case L2 -> supersystemCommand = supersystem.setDesiredState(Supersystem.SupersystemState.L2);
        case L3 -> supersystemCommand = supersystem.setDesiredState(Supersystem.SupersystemState.L3);
        case L4 -> supersystemCommand = supersystem.setDesiredState(Supersystem.SupersystemState.L4);
      }

      if (!RobotState.getInstance().useAuto()) {
        return supersystemCommand;
      }

      if (RobotState.getInstance().getCoralLevel() == L4) {
        return supersystemCommand
            .alongWith(Commands.waitUntil(supersystem::atSetpoint));
      } else {
        return supersystemCommand;
      }
    }, Set.of());
  }

  private boolean isClear(Supersystem supersystem) {
    Supersystem.SupersystemState goalState = Supersystem.SupersystemState.L2;
    RobotState.CoralLevel coralLevel = RobotState.getInstance().getCoralLevel();
    if (Objects.requireNonNull(coralLevel) == RobotState.CoralLevel.L3) {
      goalState = Supersystem.SupersystemState.L3;
    } else if (coralLevel == RobotState.CoralLevel.L4) {
      goalState = Supersystem.SupersystemState.L4;
    }
    return supersystem.atSetpoint();// && supersystem.getDesiredState() == goalState;
  }
}