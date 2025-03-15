package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ArmMovementCommandGroup extends SequentialCommandGroup {
  public ArmMovementCommandGroup(ArmSubsystem armSubsystem,
                                 ElevatorSubsystem elevatorSubsystem,
                                 double elevatorPositionInches,
                                 Rotation2d armSetpoint) {
    super(
        armSubsystem.setArmPositionFactory(ArmConstants.ARM_HOME_SETPOINT),
        armSubsystem.waitUntilAtSetpoint(),
        elevatorSubsystem.setElevatorSetpointFactory(() -> elevatorPositionInches),
        Commands.waitUntil(() -> elevatorSubsystem.atSetpoint(elevatorPositionInches)),
        armSubsystem.setArmPositionFactory(armSetpoint)
    );
  }
}