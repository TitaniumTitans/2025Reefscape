// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.algae.AlgaeIO;
import frc.robot.subsystems.algae.AlgaeIOSim;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coral.CoralIO;
import frc.robot.subsystems.coral.CoralIOTalon;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.*;
import frc.robot.supersystem.Supersystem;
import lombok.Getter;
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
  private final CoralSubsystem coralSubsystem;

  @Getter
  private final Supersystem supersystem;

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

//              algaeSubsystem = new AlgaeSubsystem(new AlgaeIO() {});
//              climberSubsystem = new ClimberSubsystem(new ClimberIOKraken());

        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOKraken());
        armSubsystem = new ArmSubsystem(new ArmIOKraken());
        algaeSubsystem = new AlgaeSubsystem(new AlgaeIO() {});
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        coralSubsystem = new CoralSubsystem(new CoralIOTalon());
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
        RobotState.getInstance().resetPose(new Pose2d(1.0, 1.0, Rotation2d.fromRadians(0.0)));

        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
        algaeSubsystem = new AlgaeSubsystem(new AlgaeIOSim());
        armSubsystem = new ArmSubsystem(new ArmIOSim());
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        coralSubsystem = new CoralSubsystem(new CoralIO() {});
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
        coralSubsystem = new CoralSubsystem(new CoralIO() {});
      }
    }

    supersystem = new Supersystem(elevatorSubsystem, armSubsystem);
    configureBindings();
    setupShuffleboardTab();
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

    supersystem.setDefaultCommand(
        supersystem.periodicCommand()
    );

//    driverController.leftTrigger()
//        .whileTrue(armSubsystem.setArmPositionFactory(ArmConstants.ARM_HOME_SETPOINT))
//        .whileFalse(Commands.runOnce(armSubsystem::setDisabled, armSubsystem));
//    driverController.rightTrigger()
//        .whileTrue(armSubsystem.setArmPositionFactory(Rotation2d.fromDegrees(-45.0)))
//        .whileFalse(Commands.runOnce(armSubsystem::setDisabled, armSubsystem));

//    driverController.leftBumper()
//        .whileTrue(elevatorSubsystem.setElevatorSetpointFactory(ElevatorConstants.HOME_SETPOINT::getValue))
//        .whileFalse(Commands.runOnce(elevatorSubsystem::setDisabled, elevatorSubsystem));
//    driverController.rightBumper()
//        .whileTrue(elevatorSubsystem.setElevatorSetpointFactory(ElevatorConstants.HOME_CLEAR_SETPOINT::getValue))
//        .whileFalse(Commands.runOnce(elevatorSubsystem::setDisabled, elevatorSubsystem));
//    driverController.a()
//        .whileTrue(elevatorSubsystem.setElevatorSetpointFactory(ElevatorConstants.L4_SETPOINT::getValue))
//        .whileFalse(Commands.runOnce(elevatorSubsystem::setDisabled, elevatorSubsystem));

    driverController.leftTrigger().onTrue(
        supersystem.setDesiredState(Supersystem.SupersystemState.HOME)
    );
    driverController.leftBumper().onTrue(
        supersystem.setDesiredState(Supersystem.SupersystemState.L2)
    );
    driverController.rightBumper().onTrue(
        supersystem.setDesiredState(Supersystem.SupersystemState.L3)
    );
    driverController.rightTrigger().onTrue(
        supersystem.setDesiredState(Supersystem.SupersystemState.L4)
    );

    driverController.povLeft()
        .whileTrue(supersystem.runArmRollers(1.5))
        .whileFalse(supersystem.runArmRollers(0.0));
    driverController.povRight()
        .whileTrue(
            supersystem.setDesiredState(Supersystem.SupersystemState.INTAKE)
                .onlyIf(() -> !elevatorSubsystem.overClearance())
                .andThen(
                    supersystem.runArmRollers(-1.5)
                        .alongWith(coralSubsystem.setScoringVoltages(3.0, 0.0, 0.0)
                            .onlyIf(() -> !elevatorSubsystem.overClearance()))))
        .whileFalse(
            supersystem.setDesiredState(Supersystem.SupersystemState.HOME)
                .onlyIf(() -> !elevatorSubsystem.overClearance())
                .andThen(
                    supersystem.runArmRollers(0.0)
                        .alongWith(coralSubsystem.setScoringVoltages(0.0, 0.0, 0.0))));

    driverController.start().onTrue(
        Commands.runOnce(() -> RobotState.getInstance().resetPose(new Pose2d()))
    );
  }

  public void setupShuffleboardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Commands");

    tab.add("Zero Elevator", Commands.runOnce(() -> elevatorSubsystem.resetElevator(0.0)));
  }
    
  public Command getAutonomousCommand()
  {
    return DriveCommands.wheelRadiusCharacterization(driveSubsystem);
  }

  public void simTick() {
    Logger.recordOutput("Sim/Simulated Robot Pose", driveSimulation.getSimulatedDriveTrainPose());
  }
}
