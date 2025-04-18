package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMPS = 0.0;
    public double[] elevatorAppliedVoltage = {0.0, 0.0};
    public double[] elevatorCurrentDraw = {0.0, 0.0};
    public boolean bottomLimitSwitch = false;
    public boolean upperLimitSwitch = false;
  }

  default void updateInputs(ElevatorIOInputsAutoLogged inputs) {}
  default void setElevatorVoltage(double voltage) {}
  default void setElevatorPosition(double positionMeters) {}
  default void resetElevatorPosition(double positionMeters) {}
}
