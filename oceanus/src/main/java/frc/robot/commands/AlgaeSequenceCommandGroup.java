package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotState;
import frc.robot.commands.swerve.SwerveDrivePIDToPose;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.supersystem.Supersystem;
import frc.robot.util.AlgaePositions;

import java.util.Objects;
import java.util.Set;

import static frc.robot.RobotState.CoralLevel.L4;

public class AlgaeSequenceCommandGroup extends SequentialCommandGroup {
  public AlgaeSequenceCommandGroup(DriveSubsystem swerve, Supersystem supersystem) {
    addCommands(
        getArmCommand(swerve, supersystem),
        new WaitUntilCommand(() -> isClear(swerve, supersystem))
            .deadlineFor(new SwerveDrivePIDToPose(swerve, swerve::getClosestClearance)
                .withClosedLoopGains(4.5, 0.0, 0.01)
                .withTolerance(Units.inchesToMeters(4.0), Units.inchesToMeters(4.0), Units.degreesToRadians(5.0)))
            .onlyIf(() -> !isClear(swerve, supersystem)),
        supersystem.runArmRollers(-1.5),
        new SwerveDrivePIDToPose(swerve, () ->
          swerve.getClosestClearance().plus(new Transform2d(0.8, 0.0, new Rotation2d()))
        )
            .withTranslationalConstraints(Units.feetToMeters(7.0), Units.feetToMeters(12))
    );

//    addCommands(
//        new WaitUntilCommand(() -> isClear(supersystem))
//            .deadlineFor(new SwerveDrivePIDToPose(swerve, swerve::getClosestClearance))
//            .alongWith(armMoveAutoScoreCommand(supersystem)),
//        new SwerveDrivePPToPose(swerve, () -> swerve.getClosestBranch(left)).getDriveCommand()
//    );
  }

  public Command getArmCommand(DriveSubsystem swerve, Supersystem supersystem) {
    return Commands.defer(() -> {
      AlgaePositions pos = swerve.findClosestAlgae();

      if (pos == AlgaePositions.AB || pos == AlgaePositions.EF ||pos == AlgaePositions.IJ) {
        return  supersystem.setDesiredState(Supersystem.SupersystemState.ALGAE_L3);
      } else {
        return supersystem.setDesiredState(Supersystem.SupersystemState.ALGAE_L2);
      }
    }, Set.of());
  }

  private boolean isClear(DriveSubsystem swerve, Supersystem supersystem) {
    Supersystem.SupersystemState state;
    AlgaePositions pos = swerve.findClosestAlgae();

    if (pos == AlgaePositions.AB || pos == AlgaePositions.EF ||pos == AlgaePositions.IJ) {
      state = Supersystem.SupersystemState.ALGAE_L3;
    } else {
      state = Supersystem.SupersystemState.ALGAE_L2;
    }

    return supersystem.atSetpoint(state) && !RobotState.getInstance().inCloseReefZone();
  }
}