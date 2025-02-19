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
import frc.robot.subsystems.intake.IntakeSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.robot.commands.AutoCommands.HumanPlayerSide;
import frc.robot.commands.AutoCommands.StartingPositions;
import frc.robot.commands.AutoCommands.CoralLocations;

import java.util.List;

public class AutoSelector {
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  public AutoSelector(DriveSubsystem drive, CoralScoralSubsystem coral, IntakeSubsystem intake) {
    NamedCommands.registerCommand("score", coral.setScorerPowerFactory(2.5).withTimeout(0.5));
    NamedCommands.registerCommand("intake", Commands.print("Waiting"));
    NamedCommands.registerCommand("zeroIntake", intake.zeroPivot());

//    autoChooser.addDefaultOption("3L1Left", AutoCommands.resetPoseAndFollowChoreoPath(drive, "3L1Left"));
//    autoChooser.addOption("L1HP", AutoCommands.resetPoseAndFollowChoreoPath(drive, "L1HP"));
    autoChooser.addOption("1L1Right", AutoCommands.resetPoseAndFollowChoreoPath(drive, "1L1Right"));
    autoChooser.addDefaultOption("2L1Right", AutoBuilder.buildAuto("2L1Right"));
    autoChooser.addOption("3L1Right", AutoBuilder.buildAuto("3L1Right"));
    autoChooser.addOption("4L1Right", AutoBuilder.buildAuto("4L1Right"));

    // parametric autos
//    autoChooser.addOption("LeftFar.IJ.K.L",
//        AutoCommands.choreoSequence(drive, coral, intake,
//            StartingPositions.StartingPosLeft,
//            HumanPlayerSide.HumanPlayerLeftFar,
//            List.of(
//                CoralLocations.I,
//                CoralLocations.K,
//                CoralLocations.L
//            )));
//
//    autoChooser.addOption("LeftClose.IJ.K.L",
//        AutoCommands.choreoSequence(drive, coral, intake,
//            StartingPositions.StartingPosLeft,
//            HumanPlayerSide.HumanPlayerLeftClose,
//            List.of(
//                CoralLocations.I,
//                CoralLocations.K,
//                CoralLocations.L
//            )));

//    autoChooser.addOption("RightFar.EF.D.C",
//        AutoCommands.choreoSequence(drive, coral, intake,
//            StartingPositions.StartingPosRight,
//            HumanPlayerSide.HumanPlayerRightFar,
//            List.of(
//                CoralLocations.E,
//                CoralLocations.D,
//                CoralLocations.C
//            )));
//
//    autoChooser.addOption("RightClose.EF.D.C",
//        AutoCommands.choreoSequence(drive, coral, intake,
//            StartingPositions.StartingPosRight,
//            HumanPlayerSide.HumanPlayerRightClose,
//            List.of(
//                CoralLocations.E,
//                CoralLocations.D,
//                CoralLocations.C
//            )));

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
