package frc.robot.subsystems.elevator;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public enum ElevatorState {
    ZEROING,
    POSITION_CONTROL,
    DISABLED
  }

  private ElevatorState goalState = ElevatorState.ZEROING;
  private double elevatorGoal = 0.0;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Elevator", inputs);

    // check for disabled robot
    if (DriverStation.isDisabled()) {
      goalState = ElevatorState.DISABLED;
    }

    Logger.recordOutput("Elevator/Goal State", goalState);

    switch (goalState) {
      case ZEROING -> {
        if (!inputs.bottomLimitSwitch) {
          io.setElevatorVoltage(0.0);
        }
      }
      case POSITION_CONTROL -> {
        io.setElevatorPosition(elevatorGoal);
      }
      case DISABLED -> {
        elevatorGoal = inputs.elevatorPosition;
        io.setElevatorPosition(elevatorGoal);
      }
    }
  }

  public void setElevatorSetpoint(DoubleSupplier goal) {
    goalState = ElevatorState.POSITION_CONTROL;
    elevatorGoal = goal.getAsDouble();
  }

  public Command setElevatorSetpointFactory(DoubleSupplier goal) {
    return runOnce(() -> setElevatorSetpoint(goal));
  }
}

