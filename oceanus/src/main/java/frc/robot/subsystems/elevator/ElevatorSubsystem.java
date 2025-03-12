package frc.robot.subsystems.elevator;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MechanismVisualizer;
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
    Logger.recordOutput("Elevator/Setpoint", elevatorGoal);

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
        elevatorGoal = inputs.elevatorPositionMeters;
        io.setElevatorPosition(elevatorGoal);
      }
    }

    MechanismVisualizer.getInstance().setElevatorHeightInches(Units.metersToInches(inputs.elevatorPositionMeters));
  }

  public void setElevatorSetpoint(DoubleSupplier goal) {
    goalState = ElevatorState.POSITION_CONTROL;
    elevatorGoal = Units.inchesToMeters(goal.getAsDouble());
  }

  public Command setElevatorSetpointFactory(DoubleSupplier goal) {
    return runOnce(() -> setElevatorSetpoint(goal));
  }

  public Command setElevatorVoltageFactory(double voltage) {
    return runEnd(
        () -> {
          if (!(inputs.elevatorPositionMeters < Units.inchesToMeters(0.5) && voltage < 0.0)) {
            io.setElevatorVoltage(voltage);
          }
        },
        () -> io.setElevatorVoltage(0.0)
    );
  }
}

