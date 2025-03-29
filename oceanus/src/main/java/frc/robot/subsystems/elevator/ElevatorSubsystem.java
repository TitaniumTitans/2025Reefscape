package frc.robot.subsystems.elevator;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.MechanismVisualizer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public enum ElevatorState {
    ZEROING,
    VOLTAGE_CONTROL,
    POSITION_CONTROL,
    DISABLED
  }

  private ElevatorState goalState = ElevatorState.DISABLED;
  private double elevatorGoal = 0.0;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Elevator", inputs);

    // check for disabled robot
    if (DriverStation.isDisabled()) {
      goalState = ElevatorState.DISABLED;
    }

    // reset elevator position
//    if (inputs.bottomLimitSwitch && inputs.elevatorPositionMeters != 0.0) {
//      io.resetElevatorPosition(0.0);
//    }

    Logger.recordOutput("Elevator/Goal State", goalState);
    Logger.recordOutput("Elevator/Setpoint", elevatorGoal);

    elevatorGoal = MathUtil.clamp(elevatorGoal, 0.0, Units.inchesToMeters(32));

    RobotState.getInstance().setSlowSpeed(
        inputs.elevatorPositionMeters >= Units.inchesToMeters(18.0)
    );

    // run state machine
    switch (goalState) {
      case ZEROING -> {
        if (!inputs.bottomLimitSwitch) {
          io.setElevatorVoltage(0.0);
        }
      }
      case POSITION_CONTROL -> io.setElevatorPosition(elevatorGoal);
      case DISABLED -> {
        elevatorGoal = inputs.elevatorPositionMeters;
        io.setElevatorPosition(elevatorGoal);
      }
      default -> {
        // we're fine here, nothing needed to be done
        return;
      }
    }

    MechanismVisualizer.getInstance().setElevatorHeightInches(Units.metersToInches(inputs.elevatorPositionMeters));
  }

  @AutoLogOutput(key = "Elevator/Over Clearance")
  public boolean overClearance() {
    return inputs.elevatorPositionMeters >= Units.inchesToMeters(ElevatorConstants.HOME_CLEAR_SETPOINT.getValue() - 1.0);
  }

  @AutoLogOutput(key = "Elevator/At L4")
  public boolean atL4() {
    return inputs.upperLimitSwitch
        || inputs.elevatorPositionMeters >= Units.inchesToMeters(ElevatorConstants.L4_SETPOINT - 5.0);
  }

  @AutoLogOutput(key = "Elevator/At Setpoint")
  public boolean atSetpoint() {
    return atSetpoint(Units.metersToInches(elevatorGoal));
  }

  public boolean atSetpoint(double setpoint) {
    return MathUtil.isNear(
        Units.inchesToMeters(setpoint),
        inputs.elevatorPositionMeters,
        Units.inchesToMeters(0.1)
    ) && elevatorGoal == Units.inchesToMeters(setpoint);
  }

  public void setDisabled() {
    goalState = ElevatorState.DISABLED;
  }

  public void resetElevator(double positionMeters) {
    io.resetElevatorPosition(positionMeters);
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
          goalState = ElevatorState.VOLTAGE_CONTROL;
          if (!(inputs.elevatorPositionMeters < Units.inchesToMeters(0.5) && voltage < 0.0)) {
            io.setElevatorVoltage(voltage);
          }
        },
        () -> io.setElevatorVoltage(0.0)
    );
  }
}

