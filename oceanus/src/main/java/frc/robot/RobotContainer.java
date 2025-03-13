// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.subsystems.algae.AlgaeIO;
import frc.robot.subsystems.algae.AlgaeIOSim;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOKraken;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;


public class RobotContainer
{
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final DriveSubsystem driveSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final AlgaeSubsystem algaeSubsystem;
    private final ArmSubsystem armSubsystem;
    private final ClimberSubsystem climberSubsystem;

    private SwerveDriveSimulation driveSimulation;
    public RobotContainer()
    {
        Logger.recordOutput("Robot Mode", Constants.getMode());

        switch (Constants.getMode()) {
          case REAL -> {
              driveSubsystem = new DriveSubsystem(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[3])
              );

              elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOKraken());
              algaeSubsystem = new AlgaeSubsystem(new AlgaeIO() {});
              armSubsystem = new ArmSubsystem(new ArmIOKraken());
              climberSubsystem = new ClimberSubsystem(new ClimberIOKraken());
          }
          case SIM -> {
            driveSimulation = new SwerveDriveSimulation(DriveConstants.MAPLE_SIM_CONFIG, new Pose2d(3, 3, new Rotation2d()));
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
            driveSubsystem = new DriveSubsystem(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3])
            );

            SimulatedArena.getInstance().resetFieldForAuto();
            RobotState.getInstance().setDriveSimulation(Optional.of(driveSimulation));

            elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
            algaeSubsystem = new AlgaeSubsystem(new AlgaeIOSim());
            armSubsystem = new ArmSubsystem(new ArmIOSim());
            climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
          }
          default -> {
            driveSubsystem = new DriveSubsystem(
                new GyroIO() {
                },
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {}
            );

            elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {});
            armSubsystem = new ArmSubsystem(new ArmIO() {});
            algaeSubsystem = new AlgaeSubsystem(new AlgaeIO() {});
            climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
          }
        }

      configureBindings();
    }
    
    
  private void configureBindings() {
//        driveSubsystem.setDefaultCommand(
//            DriveCommands.joystickDrive(
//                driveSubsystem,
//                () -> -driverController.getLeftY(),
//                () -> -driverController.getLeftX(),
//                () -> -driverController.getRightX()
//            )
//        );

    ElevatorPositionCommand elevatorCommand = new ElevatorPositionCommand(
        algaeSubsystem,
        elevatorSubsystem,
        armSubsystem
    );

      driverController.a().toggleOnTrue(Commands.startEnd(
          () -> elevatorCommand.getCommand(ElevatorPositionCommand.ScoringPose.L4).schedule(),
          () -> elevatorCommand.getCommand(ElevatorPositionCommand.ScoringPose.HOME).schedule()
      ));

      driverController.x().toggleOnTrue(Commands.startEnd(
          () -> elevatorSubsystem.setElevatorSetpointFactory(ElevatorConstants.L4_SETPOINT::getValue).schedule(),
          () -> elevatorSubsystem.setElevatorSetpointFactory(ElevatorConstants.HOME_SETPOINT::getValue).schedule()
      ));

       driverController.leftBumper().whileTrue(elevatorSubsystem.setElevatorVoltageFactory(1.5));
       driverController.rightBumper().whileTrue(elevatorSubsystem.setElevatorVoltageFactory(-1.5));

       driverController.leftTrigger().whileTrue(armSubsystem.setArmVoltageFactory(1.5));
       driverController.rightTrigger().whileTrue(armSubsystem.setArmVoltageFactory(-1.5));
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }

    public void simTick() {
        Logger.recordOutput("Sim/Simulated Robot Pose", driveSimulation.getSimulatedDriveTrainPose());
    }
}
