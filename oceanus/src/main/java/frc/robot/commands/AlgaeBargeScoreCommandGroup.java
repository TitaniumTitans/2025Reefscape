package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ChoreoPoses;
import frc.robot.commands.swerve.SwerveDrivePIDToPose;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.supersystem.Supersystem;

public class AlgaeBargeScoreCommandGroup extends SequentialCommandGroup {
  public AlgaeBargeScoreCommandGroup(DriveSubsystem swerve, Supersystem supersystem) {
    super(
        swerve.driveToPose(ChoreoPoses.NET_CLEAR),
        supersystem.setDesiredState(Supersystem.SupersystemState.BARGE),
        new SwerveDrivePIDToPose(swerve, ChoreoPoses.BLUE_NET.getPose())
            .withTranslationalConstraints(0.5, 0.5)
    );
  }
}