// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.algae.AlgaeIOSim;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;


public class RobotContainer
{
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    DriveSubsystem driveSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    AlgaeSubsystem algaeSubsystem;

    private SwerveDriveSimulation driveSimulation;
    public RobotContainer()
    {
        switch (Constants.getMode()) {
          case REAL -> {
              driveSubsystem = new DriveSubsystem(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[3])
              );
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
          }
          case REPLAY -> {
              driveSubsystem = new DriveSubsystem(
                  new GyroIO() {
                  },
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {}
              );

              elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {});
          }
        }

        configureBindings();
    }
    
    
    private void configureBindings() {
        driveSubsystem.setDefaultCommand(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()
            )
        );

        driverController.a().whileTrue(
            elevatorSubsystem.setElevatorSetpointFactory(ElevatorConstants.L4_SETPOINT::getValue)
        ).onFalse(
            elevatorSubsystem.setElevatorSetpointFactory(ElevatorConstants.HOME_SETPOINT::getValue)
        );

        driverController.b().whileTrue(
            elevatorSubsystem.setElevatorSetpointFactory(ElevatorConstants.L3_SETPOINT::getValue)
        ).onFalse(
            elevatorSubsystem.setElevatorSetpointFactory(ElevatorConstants.HOME_SETPOINT::getValue)
        );

       driverController.x().whileTrue(
           algaeSubsystem.setAlgaeAngle(45)
       ).whileFalse(
           algaeSubsystem.setAlgaeAngle(90.0)
       );
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }

    public void simTick() {
        Logger.recordOutput("Sim/Simulated Robot Pose", driveSimulation.getSimulatedDriveTrainPose());
    }
}
