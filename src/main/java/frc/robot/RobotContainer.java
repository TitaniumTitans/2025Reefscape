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
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOSparkMax;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  private SwerveDriveSimulation driveSimulation = null;
  private final CommandXboxController driveController = new CommandXboxController(0);

  DriveSubsystem driveSubsystem;
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL ->
        driveSubsystem = new DriveSubsystem(
            new GyroIOPigeon2(),
            new ModuleIOSparkMax(DriveConstants.MODULE_CONSTANTS[0]),
            new ModuleIOSparkMax(DriveConstants.MODULE_CONSTANTS[1]),
            new ModuleIOSparkMax(DriveConstants.MODULE_CONSTANTS[2]),
            new ModuleIOSparkMax(DriveConstants.MODULE_CONSTANTS[3])
        );

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
      }
      case REPLAY ->
        driveSubsystem = new DriveSubsystem(
            new GyroIO() {
            },
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {}
        );
    }
    configureBindings();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(DriveCommands.joystickDrive(
        driveSubsystem,
        () -> -driveController.getLeftY(),
        () -> -driveController.getLeftX(),
        () -> -driveController.getRightX()
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void simTick() {
    Logger.recordOutput("Sim/Simulated Robot Pose", driveSimulation.getSimulatedDriveTrainPose());
  }
}
