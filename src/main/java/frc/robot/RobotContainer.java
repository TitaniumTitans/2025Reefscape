// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOSparkMax;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOProto;
import frc.robot.subsystems.intake.IntakeSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  private SwerveDriveSimulation driveSimulation;
  private VisionSubsystem visionSubsystem;
  private final CommandXboxController driveController = new CommandXboxController(0);

  DriveSubsystem driveSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL -> {
        driveSubsystem = new DriveSubsystem(
            new GyroIOPigeon2(),
            new ModuleIOSparkMax(DriveConstants.MODULE_CONSTANTS[0]),
            new ModuleIOSparkMax(DriveConstants.MODULE_CONSTANTS[1]),
            new ModuleIOSparkMax(DriveConstants.MODULE_CONSTANTS[2]),
            new ModuleIOSparkMax(DriveConstants.MODULE_CONSTANTS[3])
        );
        intakeSubsystem = new IntakeSubsystem(new IntakeIOProto());
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

        RobotState.getInstance().setDriveSimulation(driveSimulation);

        visionSubsystem = new VisionSubsystem(
            VisionConstants.FILTER_PARAMETERS,
            new VisionIOPhotonSimulation(
                "BACK_RIGHT",
                VisionConstants.BACK_RIGHT_TRANSFORM,
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                VisionConstants.SIM_CAMERA_PROPERTIES
            ),
            new VisionIOPhotonSimulation(
                "BACK_LEFT",
                VisionConstants.BACK_LEFT_TRANSFORM,
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                VisionConstants.SIM_CAMERA_PROPERTIES
            ));

        VisionEnvironmentSimulator.getInstance().addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
        VisionEnvironmentSimulator.getInstance().addRobotPoseSupplier(
            RobotBase.isReal() ?
                RobotState.getInstance()::getEstimatedPose :
                driveSimulation::getSimulatedDriveTrainPose
        );

        SimulatedArena.getInstance().resetFieldForAuto();

        intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});
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
        intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});
      }
      default -> {
        intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});
      }
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

    visionSubsystem.setDefaultCommand(visionSubsystem.processVision(RobotBase.isReal() ?
        RobotState.getInstance()::getEstimatedPose :
        driveSimulation::getSimulatedDriveTrainPose));

    if (RobotBase.isSimulation()) {
      driveController.start().onTrue(
          Commands.runOnce(() -> driveSimulation
              .setSimulationWorldPose(
                  RobotState.getInstance().getEstimatedPose())));
    }

    driveController.a().whileTrue(
        Commands.run(() -> intakeSubsystem.setIntakePower(6.0), intakeSubsystem)
    ).whileFalse(
        Commands.run(() -> intakeSubsystem.setIntakePower(0.0))
    );
    driveController.b().whileTrue(
        Commands.run(() -> intakeSubsystem.setIntakePower(-6.0), intakeSubsystem)
    ).whileFalse(
        Commands.run(() -> intakeSubsystem.setIntakePower(0.0))
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void simTick() {
    Logger.recordOutput("Sim/Simulated Robot Pose", driveSimulation.getSimulatedDriveTrainPose());
  }
}
