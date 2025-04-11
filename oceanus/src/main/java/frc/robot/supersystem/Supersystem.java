package frc.robot.supersystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

public class Supersystem extends SubsystemBase {


  public enum SupersystemState {
    DISABLED,
    INTAKE,
    HOME,
    L2,
    L3,
    L4,
    L4_OVERRIDE,
    BARGE,
    ALGAE_L2,
    ALGAE_L3
  }
  private final ElevatorSubsystem elevatorSubsystem;

  private final ArmSubsystem armSubsystem;
  @AutoLogOutput(key = "Supersystem State")
  @Getter
  private SupersystemState desiredState = SupersystemState.DISABLED;

  @Setter
  private double rollerVoltage = 0.0;

  public Supersystem(
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem
  ) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;

    AutoLogOutputManager.addObject(this);
  }

  public Command setDesiredState(SupersystemState state) {
    return Commands.runOnce(() -> desiredState = state);
  }

  public Command runArmRollers(double voltage) {
    return Commands.runOnce(() -> rollerVoltage = voltage);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || desiredState == SupersystemState.DISABLED) {
      desiredState = SupersystemState.DISABLED;
    }
  }

  @AutoLogOutput(key = "RobotState/Has Coral")
  public boolean hasCoral() {
    return armSubsystem.hasCoral();
  }

  @AutoLogOutput(key = "Supersystem/At Setpoint")
  public boolean atSetpoint() {
    return armSubsystem.atSetpoint()
        && elevatorSubsystem.atSetpoint();
  }

  public boolean atSetpoint(RobotState.CoralLevel coralLevel) {
    switch (coralLevel) {
      case L2 -> {
        return armSubsystem.atSetpoint(ArmConstants.L2_SETPOINT.getDegrees())
            && elevatorSubsystem.atSetpoint(ElevatorConstants.L2_SETPOINT.getValue());
      }
      case L3 -> {
        return armSubsystem.atSetpoint(ArmConstants.L3_SETPOINT.getDegrees())
            && elevatorSubsystem.atSetpoint(ElevatorConstants.L3_SETPOINT.getValue());
      }
      case L4 -> {
        return armSubsystem.atSetpoint(ArmConstants.L4_SETPOINT.getDegrees())
            && elevatorSubsystem.atSetpoint(ElevatorConstants.L4_SETPOINT);
      }
    }

    return false;
  }

  public boolean atSetpoint(SupersystemState state) {
    switch (state) {
      case DISABLED -> {
        return true;
      }
      case INTAKE -> {
        return armSubsystem.atSetpoint(ArmConstants.INTAKE_SETPOINT.getDegrees())
            && elevatorSubsystem.atSetpoint(ElevatorConstants.INTAKE_SETPOINT);
      }
      case HOME -> {
        return armSubsystem.atSetpoint(ArmConstants.ARM_HOME_SETPOINT.getDegrees())
            && elevatorSubsystem.atSetpoint(ElevatorConstants.HOME_SETPOINT.getValue());
      }
      case L2 -> {
        return armSubsystem.atSetpoint(ArmConstants.L2_SETPOINT.getDegrees())
            && elevatorSubsystem.atSetpoint(ElevatorConstants.L2_SETPOINT.getValue());
      }
      case L3 -> {
        return armSubsystem.atSetpoint(ArmConstants.L3_SETPOINT.getDegrees())
            && elevatorSubsystem.atSetpoint(ElevatorConstants.L3_SETPOINT.getValue());
      }
      case L4 -> {
        return armSubsystem.atSetpoint(ArmConstants.L4_SETPOINT.getDegrees())
            && elevatorSubsystem.atSetpoint(ElevatorConstants.L4_SETPOINT);
      }
      case L4_OVERRIDE -> {
        return armSubsystem.atSetpoint(ArmConstants.L4_SETPOINT.getDegrees())
            && elevatorSubsystem.atSetpoint(ElevatorConstants.L4_SETPOINT);
      }
      case BARGE -> {
        return armSubsystem.atSetpoint(ArmConstants.BARGE_SETPOINT.getDegrees())
            && elevatorSubsystem.atSetpoint(ElevatorConstants.BARGE_SETPOINT);
      }
      case ALGAE_L2 -> {
        return armSubsystem.atSetpoint(ArmConstants.ALGAE_SETPOINT.getDegrees())
            && elevatorSubsystem.atSetpoint(ElevatorConstants.ALGAE_L2_SETPOINT.getValue());
      }
      case ALGAE_L3 -> {
        return armSubsystem.atSetpoint(ArmConstants.ALGAE_SETPOINT.getDegrees())
            && elevatorSubsystem.atSetpoint(ElevatorConstants.ALGAE_L3_SETPOINT.getValue());
      }
    }

    return false;
  }

  public Command periodicCommand() {
    return Commands.run(() -> {
      // run arm roller
      if ((RobotState.getInstance().isSlowSpeed() && armSubsystem.atSetpoint(ArmConstants.L4_SETPOINT.getDegrees()))) {
        rollerVoltage = MathUtil.clamp(rollerVoltage, -0.75, 12.0);
      }
      armSubsystem.setArmRollerVoltage(rollerVoltage);

      // L4 override takes all priority
      if (desiredState == SupersystemState.L4_OVERRIDE) {
        armSubsystem.setArmPosition(ArmConstants.L4_SETPOINT);
        elevatorSubsystem.setElevatorSetpoint(() -> ElevatorConstants.L4_SETPOINT);
        return;
      }

      if (RobotState.getInstance().inCloseReefZone()) {
        return;
      }

      if (DriverStation.isDisabled() || desiredState == SupersystemState.DISABLED) {
        desiredState = SupersystemState.DISABLED;
        elevatorSubsystem.setDisabled();
        armSubsystem.setDisabled();
        return;
      }

      if (RobotState.getInstance().useAuto()) {
        // if we are coming or leaving home, go to a clearance state
        if (RobotState.getInstance().inFarReefZone()) {
          if (desiredState == SupersystemState.L4
              && !elevatorSubsystem.atSetpoint(ElevatorConstants.L4_SETPOINT)) {
            // we're at L4, and we're in the reef location
            elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.L3_SETPOINT::getValue);
            armSubsystem.setArmPosition(ArmConstants.L3_SETPOINT);
            return;
          } else if (elevatorSubsystem.atSetpoint(ElevatorConstants.L4_SETPOINT)) {
            // we're going to L4, and in the reef
            elevatorSubsystem.setElevatorSetpoint(() -> ElevatorConstants.L4_SETPOINT);
            armSubsystem.setArmPosition(ArmConstants.L4_SETPOINT);
            return;
          }
        } else if (RobotState.getInstance().inBargeZone()) {
          if (desiredState == SupersystemState.BARGE
              && !elevatorSubsystem.atSetpoint(ElevatorConstants.BARGE_SETPOINT)) {
            // we're at L4, and we're in the reef location
            elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.L3_SETPOINT::getValue);
            armSubsystem.setArmPosition(ArmConstants.L3_SETPOINT);
            return;
          } else if (elevatorSubsystem.atSetpoint(ElevatorConstants.BARGE_SETPOINT)) {
            // we're going to L4, and in the reef
            elevatorSubsystem.setElevatorSetpoint(() -> ElevatorConstants.BARGE_SETPOINT);
            armSubsystem.setArmPosition(ArmConstants.BARGE_SETPOINT);
            return;
          }
        } else if (
            (desiredState == SupersystemState.HOME || desiredState == SupersystemState.INTAKE)
                && elevatorSubsystem.overClearance() && !armSubsystem.atHome()) {
          elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.HOME_CLEAR_SETPOINT::getValue);
          armSubsystem.setArmPosition(ArmConstants.ARM_HOME_SETPOINT);
          // wait until everything is clear
          return;
        }
      }

      // Everything else passes, so we can move the arm and elevator to the desired locations
      switch (desiredState) {
        case DISABLED -> {
          elevatorSubsystem.setDisabled();
          armSubsystem.setDisabled();
        }
        case INTAKE -> {
          elevatorSubsystem.setElevatorSetpoint(() -> ElevatorConstants.INTAKE_SETPOINT);
          armSubsystem.setArmPosition(ArmConstants.INTAKE_SETPOINT);
        }
        case HOME -> {
          elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.HOME_SETPOINT::getValue);
          armSubsystem.setArmPosition(ArmConstants.ARM_HOME_SETPOINT);
        }
        case L2 -> {
          elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.L2_SETPOINT::getValue);
          armSubsystem.setArmPosition(ArmConstants.L2_SETPOINT);
        }
        case L3 -> {
          elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.L3_SETPOINT::getValue);
          armSubsystem.setArmPosition(ArmConstants.L3_SETPOINT);
        }
        case L4 -> {
          elevatorSubsystem.setElevatorSetpoint(() -> ElevatorConstants.L4_SETPOINT);
          armSubsystem.setArmPosition(ArmConstants.L4_SETPOINT);
        }
        case BARGE -> {
          elevatorSubsystem.setElevatorSetpoint(() -> ElevatorConstants.BARGE_SETPOINT);
          armSubsystem.setArmPosition(ArmConstants.BARGE_SETPOINT);
        }
        case ALGAE_L2 -> {
          elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.ALGAE_L2_SETPOINT::getValue);
          armSubsystem.setArmPosition(ArmConstants.ALGAE_SETPOINT);
        }
        case ALGAE_L3 -> {
          elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.ALGAE_L3_SETPOINT::getValue);
          armSubsystem.setArmPosition(ArmConstants.ALGAE_SETPOINT);
        }
      }

      // when leaving home watch for arm while moving up
      if (!elevatorSubsystem.overClearance()
          && desiredState != SupersystemState.HOME
          && desiredState != SupersystemState.INTAKE) {
        armSubsystem.setArmPosition(ArmConstants.ARM_HOME_SETPOINT);
      }
    }, this, armSubsystem, elevatorSubsystem);
  }
}
