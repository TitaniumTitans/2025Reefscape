// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlgaeBargeScoreCommandGroup;
import frc.robot.commands.AlgaeSequenceCommandGroup;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreSequenceCommandGroup;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.commands.auto.AutoSelector;
import frc.robot.commands.coral.CoralResetCommand;
import frc.robot.commands.swerve.SwerveDrivePIDToPose;
import frc.robot.subsystems.algae.AlgaeIO;
import frc.robot.subsystems.algae.AlgaeIOSim;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coral.CoralIO;
import frc.robot.subsystems.coral.CoralIOTalon;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonReal;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.supersystem.Supersystem;
import lombok.Getter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.Set;

import static frc.robot.RobotState.CoralLevel.*;


public class RobotContainer
{
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final DriveSubsystem swerve;
  private final ElevatorSubsystem elevatorSubsystem;
  private final AlgaeSubsystem algaeSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final CoralSubsystem coralSubsystem;
  private VisionSubsystem visionSubsystem;

  @Getter
  private final Supersystem supersystem;

  private final AutoSelector autoSelector;

  private SwerveDriveSimulation driveSimulation;

  private final Command scoreDriveLeft;
  private final Command scoreDriveRight;
  private final Command clearanceDrive;

  public RobotContainer()
  {
    Logger.recordOutput("Robot Mode", Constants.getMode());

    AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    switch (Constants.getMode()) {
      case REAL -> {
        swerve = new DriveSubsystem(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[0]),
            new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[1]),
            new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[2]),
            new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[3])
        );
        
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOKraken());
        armSubsystem = new ArmSubsystem(new ArmIOKraken());
        algaeSubsystem = new AlgaeSubsystem(new AlgaeIO() {});
        coralSubsystem = new CoralSubsystem(new CoralIOTalon());
        climberSubsystem = new ClimberSubsystem(new ClimberIOKraken());

//        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {});
//        armSubsystem = new ArmSubsystem(new ArmIO() {});
//        coralSubsystem = new CoralSubsystem(new CoralIO() {});
//        algaeSubsystem = new AlgaeSubsystem(new AlgaeIO() {});

        visionSubsystem = new VisionSubsystem(
            VisionConstants.FILTER_PARAMETERS,
            new VisionIOPhotonReal("RightCamera", VisionConstants.RIGHT_CAMERA_TRANSFORM, tagFieldLayout),
            new VisionIOPhotonReal("LeftCamera", VisionConstants.LEFT_CAMERA_TRANSFORM, tagFieldLayout)
        );
      }
      case SIM -> {
        driveSimulation = new SwerveDriveSimulation(DriveConstants.MAPLE_SIM_CONFIG, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        swerve = new DriveSubsystem(
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
        swerve = new DriveSubsystem(
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

    clearanceDrive = new SwerveDrivePIDToPose(swerve, swerve::getClosestClearance);
    scoreDriveLeft = new SwerveDrivePIDToPose(swerve, () -> swerve.getClosestBranch(true));
    scoreDriveRight = new SwerveDrivePIDToPose(swerve, () -> swerve.getClosestBranch(false));

    supersystem = new Supersystem(elevatorSubsystem, armSubsystem);
    autoSelector = new AutoSelector(swerve, supersystem, coralSubsystem);
    configureBindings();
    setupShuffleboardTab();
  }
    
    
  private void configureBindings() {
    swerve.setDefaultCommand(
        DriveCommands.joystickDrive(
            swerve,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()
        )
    );

    if (Robot.isReal()) {
      visionSubsystem.setDefaultCommand(
          visionSubsystem.processVision(RobotState.getInstance()::getEstimatedPose)
              .ignoringDisable(true)
      );
    }

    supersystem.setDefaultCommand(
        supersystem.periodicCommand()
    );

//    driverController.leftBumper()
//        .whileTrue(
//            autoScoreCommand(true)
//        );

//    driverController.rightBumper()
//        .whileTrue(
//            autoScoreCommand(false)
//        );

    // Driver controls

    // Hopper
    driverController.rightTrigger().whileTrue(
        supersystem.setDesiredState(Supersystem.SupersystemState.INTAKE)
            .andThen(supersystem.runArmRollers(-3.5)
                .alongWith(coralSubsystem.setScoringVoltages(6.0, 3.0, 3.0)))
    ).whileFalse(
        supersystem.setDesiredState(Supersystem.SupersystemState.HOME)
            .andThen(supersystem.runArmRollers(0.0)
                .alongWith(coralSubsystem.setScoringVoltages(0.0, 0.0, 0.0)))
    );
    driverController.leftTrigger().whileTrue(
        coralSubsystem.setPivotAngle(125)
            .andThen(coralSubsystem.setScoringVoltages(0.0, -3.0, 0.0))
    ).onFalse(
        coralSubsystem.setPivotAngle(125)
            .andThen(coralSubsystem.setScoringVoltages(0.0, 0.0, 0.0))
    );

    // arm rollers
    driverController.leftBumper().whileTrue(
        new ConditionalCommand(
            supersystem.runArmRollers(1.5),
            supersystem.runArmRollers(12.0),
            armSubsystem::hasCoral
        )
    ).whileFalse(
        supersystem.runArmRollers(0.0)
    );
    driverController.rightBumper().whileTrue(
        supersystem.runArmRollers(-1.5)
    ).whileFalse(
        supersystem.runArmRollers(-0.75)
    );

    // ground intake
    driverController.a()
        .whileTrue(
            new ConditionalCommand(
            coralSubsystem.setPivotAngle(93)
                .andThen(coralSubsystem.setScoringVoltages(0.0, -2.5, 0.0)),
                coralSubsystem.setPivotAngle(125)
                    .andThen(coralSubsystem.setScoringVoltages(0.0, 3.0, 0.0)),
                coralSubsystem::hasCoral)
        ).whileFalse(
            coralSubsystem.setPivotAngle(93)
                .andThen(coralSubsystem.setScoringVoltages(0.0, 0.0, 0.0))
        );
    driverController.b()
        .whileTrue(
            coralSubsystem.setPivotAngle(190)
                .andThen(coralSubsystem.setScoringVoltages(0.0, 4.0, 0.0))
        ).whileFalse(
            coralSubsystem.setPivotAngle(93)
                .andThen(coralSubsystem.setScoringVoltages(0.0, 1.5, 0.0))
        );

    // trigger auto alignment
//    driverController.x()
//        .whileTrue(
//            new SwerveDrivePIDToPose(swerve, swerve::getClosestClearance)
//                .withTranslationalConstraints(
//                    Units.feetToMeters(12.0), Units.feetToMeters(12.0 / 0.125))
//                .andThen(armMoveAutoScoreCommand())
//                .andThen(new SwerveDrivePIDToPose(swerve, () -> swerve.getClosestBranch(true)))
//        );
    driverController.x()
        .whileTrue(
            new ScoreSequenceCommandGroup(swerve, supersystem, true)
        );
    driverController.y()
        .whileTrue(
            new ScoreSequenceCommandGroup(swerve, supersystem, false)
        );

    // resets
    driverController.start().onTrue(
        Commands.runOnce(() -> RobotState.getInstance().resetPose(new Pose2d()))
    );
    driverController.povUp()
        .whileTrue(new CoralResetCommand(coralSubsystem))
        .whileFalse(coralSubsystem.setPivotVoltageFactory(0.0));
    driverController.povLeft()
        .whileTrue(swerve.driveToPose(ChoreoPoses.STARTING_POS_LEFT::getPose));
    driverController.povDown()
        .whileTrue(coralSubsystem.setPivotVoltageFactory(-1.0))
        .whileFalse(coralSubsystem.setPivotVoltageFactory(0.0));
    driverController.povRight()
        .whileTrue(swerve.driveToPose(ChoreoPoses.STARTING_POS_RIGHT::getPose));

    // operator controls
    // set scoring level
    operatorController.x().onTrue(
        Commands.runOnce(() -> RobotState.getInstance().setCoralLevel(L2))
            .ignoringDisable(true)
    );
    operatorController.y().onTrue(
        Commands.runOnce(() -> RobotState.getInstance().setCoralLevel(L3))
            .ignoringDisable(true)
    );
    operatorController.b().onTrue(
        Commands.runOnce(() -> RobotState.getInstance().setCoralLevel(L4))
            .ignoringDisable(true)
    );

    // manual setpoints
    operatorController.leftTrigger()
        .onTrue(supersystem.setDesiredState(Supersystem.SupersystemState.HOME));
    operatorController.rightTrigger()
        .onTrue(supersystem.setDesiredState(Supersystem.SupersystemState.L4_OVERRIDE));
//    operatorController.a()
//        .onTrue(supersystem.setDesiredState(Supersystem.SupersystemState.BARGE));
    operatorController.a()
            .onTrue(new AlgaeBargeScoreCommandGroup(swerve, supersystem));

    operatorController.povLeft().whileTrue(
        supersystem.setDesiredState(Supersystem.SupersystemState.INTAKE)
            .andThen(coralSubsystem.setScoringVoltages(-3.0, -3.0, -3.0)
                .alongWith(supersystem.runArmRollers(1.0)))
    ).onFalse(
        supersystem.setDesiredState(Supersystem.SupersystemState.HOME)
            .andThen(coralSubsystem.setScoringVoltages(0.0, 0.0, 0.0)
                .alongWith(supersystem.runArmRollers(0.0)))
    );

    // Algae removal
    operatorController.povUp().whileTrue(
        new AlgaeSequenceCommandGroup(swerve, supersystem)
    );

    //climber controls
    operatorController.leftBumper()
            .whileTrue(climberSubsystem.setClimberPosition(170));
    operatorController.rightBumper()
        .whileTrue(
            climberSubsystem.setClimberPowerFactory(3.0)
        ).whileFalse(
            climberSubsystem.setClimberPowerFactory(0.0)
        );

    operatorController.povRight()
        .whileTrue(climberSubsystem.setClimberPowerFactory(-3.0))
        .whileFalse(climberSubsystem.setClimberPowerFactory(0.0));
  }

  public void setupShuffleboardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Commands");

    tab.add("Zero Elevator", Commands.runOnce(() -> elevatorSubsystem.resetElevator(0.0)).ignoringDisable(true));
    tab.add("Zero Coral Pivot", coralSubsystem.resetPivotFactory());
    tab.add("Auto Hopper", AutoCommands.intakeUntilCoral(coralSubsystem, supersystem));

    tab.add("StartRightE Path", AutoCommands.followChoreoPath("RightE"));
  }
    
  public Command getAutonomousCommand()
  {
    return autoSelector.getAutoCommand();
  }

  public void simTick() {
    Logger.recordOutput("Sim/Simulated Robot Pose", driveSimulation.getSimulatedDriveTrainPose());
  }

  public Command autoScoreCommand(boolean left) {
    return Commands.defer(() -> {
      Command score = left ? scoreDriveLeft.asProxy() : scoreDriveRight.asProxy();
      Command supersystemCommand = Commands.none();
      switch (RobotState.getInstance().getCoralLevel()) {
        case L2 -> supersystemCommand = supersystem.setDesiredState(Supersystem.SupersystemState.L2);
        case L3 -> supersystemCommand = supersystem.setDesiredState(Supersystem.SupersystemState.L3);
        case L4 -> supersystemCommand = supersystem.setDesiredState(Supersystem.SupersystemState.L4);
      }

      if (!RobotState.getInstance().useAuto()) {
        return supersystemCommand;
      }

      if (RobotState.getInstance().getCoralLevel() == L4) {
        return clearanceDrive
            .andThen(supersystemCommand
                .alongWith(Commands.waitUntil(supersystem::atSetpoint)))
            .andThen(score);
      } else {
        return clearanceDrive
            .andThen(supersystemCommand
                .alongWith(score));
      }
    }, Set.of(swerve));
  }

  public Command armMoveAutoScoreCommand() {
    return Commands.defer(() -> {
      Command supersystemCommand = Commands.none();
      switch (RobotState.getInstance().getCoralLevel()) {
        case L2 -> supersystemCommand = supersystem.setDesiredState(Supersystem.SupersystemState.L2);
        case L3 -> supersystemCommand = supersystem.setDesiredState(Supersystem.SupersystemState.L3);
        case L4 -> supersystemCommand = supersystem.setDesiredState(Supersystem.SupersystemState.L4);
      }

      if (!RobotState.getInstance().useAuto()) {
        return supersystemCommand;
      }

      if (RobotState.getInstance().getCoralLevel() == L4) {
        return supersystemCommand
                .alongWith(Commands.waitUntil(supersystem::atSetpoint));
      } else {
        return supersystemCommand;
      }
    }, Set.of());
  }
}
