// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoSelector;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coralscoral.CoralScoralIO;
import frc.robot.subsystems.coralscoral.CoralScoralIOTalon;
import frc.robot.subsystems.coralscoral.CoralScoralSimulation;
import frc.robot.subsystems.coralscoral.CoralScoralSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class RobotContainer {
  private SwerveDriveSimulation driveSimulation;
  private VisionSubsystem visionSubsystem;
  private final CommandXboxController driveController = new CommandXboxController(0);

  private final DriveSubsystem driveSubsystem;
  private final CoralScoralSubsystem coralSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private final AutoSelector autoSelector;

  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL -> {
        driveSubsystem = new DriveSubsystem(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[0]),
            new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[1]),
            new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[2]),
            new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[3])
        );
        intakeSubsystem = new IntakeSubsystem(new IntakeIOTalon());
        coralSubsystem = new CoralScoralSubsystem(new CoralScoralIOTalon());
        climberSubsystem = new ClimberSubsystem(new ClimberIOKraken());
        visionSubsystem = new VisionSubsystem(
            VisionConstants.FILTER_PARAMETERS,
            new VisionIOPhotonReal("BackCamera",
                VisionConstants.BACK_CAMERA_TRANSFORM,
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField))
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

//        visionSubsystem = new VisionSubsystem(
//            VisionConstants.FILTER_PARAMETERS,
//            new VisionIOPhotonSimulation(
//                "SIM",
//                VisionConstants.SIM_CAMERA_TRANSFORM,
//                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
//                VisionConstants.SIM_CAMERA_PROPERTIES
//            ));


        VisionEnvironmentSimulator.getInstance().addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
        VisionEnvironmentSimulator.getInstance().addRobotPoseSupplier(RobotState.getInstance()::getEstimatedPose);

        SimulatedArena.getInstance().resetFieldForAuto();

//        RobotState.getInstance().setDriveSimulation(Optional.of(driveSimulation));

        intakeSubsystem = new IntakeSubsystem(new IntakeIOSimulation());
        coralSubsystem = new CoralScoralSubsystem(new CoralScoralSimulation());
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
        intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});
        coralSubsystem = new CoralScoralSubsystem(new CoralScoralIO() {});
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
      }
    }

    autoSelector = new AutoSelector(driveSubsystem, coralSubsystem, intakeSubsystem);

    configureBindings();
    setShuffleboardCommands();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(DriveCommands.joystickDrive(
        driveSubsystem,
        () -> -driveController.getLeftY(),
        () -> -driveController.getLeftX(),
        () -> -driveController.getRightX()
    ));

    visionSubsystem.setDefaultCommand(visionSubsystem.processVision(
        RobotState.getInstance()::getEstimatedPose
    ).ignoringDisable(true));

//    coralSubsystem.setDefaultCommand(coralSubsystem.holdPositionFactory());
//    intakeSubsystem.setDefaultCommand(intakeSubsystem.zeroPivot(0.6));

//    coralSubsystem.setDefaultCommand(
//        coralSubsystem.setScorerPowerFactory(0.175)
//            .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
//    );

    driveController.rightBumper().whileTrue(
        coralSubsystem.setPivotPowerFactory(-2.0)
    ).onFalse(
        coralSubsystem.holdPositionFactory()
    );

    driveController.leftBumper().whileTrue(
        coralSubsystem.setPivotPowerFactory(2.0)
    ).onFalse(
        coralSubsystem.holdPositionFactory()
    );

    driveController.b().whileTrue(
        coralSubsystem.setScorerPowerFactory(-1.0)
            .withTimeout(0.1)
            .andThen(coralSubsystem.setScorerPowerFactory(1.85
            ))
    );
    driveController.a().whileTrue(
        coralSubsystem.setScorerPowerFactory(2.0)
    );

//    driveController.y().whileTrue(
//        intakeSubsystem.pickUpAlgea()
//    );

    driveController.x().whileTrue(
        intakeSubsystem.dropAlgae()
    ).onFalse(intakeSubsystem.setPivotPositionFactory(100)
        .withTimeout(0.5)
        .andThen(intakeSubsystem.zeroPivot()));

    driveController.y().onTrue(
        driveSubsystem.resetPoseFactory(
            ChoreoPoses.A_B)
    );

    driveController.rightTrigger().whileTrue(
        climberSubsystem.setClimberPowerFactory(12.0)
    );
    driveController.leftTrigger().whileTrue(
        climberSubsystem.setClimberPowerFactory(-12.0)
    );

    driveController.start().onTrue(
        driveSubsystem.resetPoseFactory(
            new Pose2d(RobotState.getInstance().getEstimatedPose().getTranslation(),
            new Rotation2d())
        )
    );

    driveController.povUp()
        .whileTrue(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> 0.5,
                () -> 0.0,
                () -> 0.0
            )
        ).onFalse(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> 0.0,
                () -> 0.0,
                () -> 0.0
            )
        );

    driveController.povDown()
        .whileTrue(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> -0.5,
                () -> 0.0,
                () -> 0.0
            )
        ).onFalse(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> 0.0,
                () -> 0.0,
                () -> 0.0
            )
        );

    driveController.povRight()
        .whileTrue(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> 0.0,
                () -> 0.5,
                () -> 0.0
            )
        ).onFalse(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> 0.0,
                () -> 0.0,
                () -> 0.0
            )
        );
    driveController.povLeft()
        .whileTrue(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> 0.0,
                () -> -0.5,
                () -> 0.0
            )
        ).onFalse(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> 0.0,
                () -> 0.0,
                () -> 0.0
            )
        );
  }

  public Command getAutonomousCommand() {
    return autoSelector.getAutoCommand();
  }

  public void setShuffleboardCommands() {
    ShuffleboardTab commands = Shuffleboard.getTab("Commands");

    commands.add("Reset Pivot Position", intakeSubsystem.zeroPivot().ignoringDisable(true));
  }

  public void simTick() {
    Logger.recordOutput("Sim/Simulated Robot Pose", driveSimulation.getSimulatedDriveTrainPose());
  }
}
