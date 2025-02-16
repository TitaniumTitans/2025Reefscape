package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public double elevatorPosition = 0.0;
    public double elevatorVelocity = 0.0;
    public double elevatorAppliedVoltage = 0.0;
    public double[] elevatorCurrentDraw = {0.0, 0.0};
    public boolean bottomLimitSwitch = false;
  }

  default void updateInputs(ElevatorIOInputsAutoLogged inputs) {}
  default void setElevatorVoltage(double voltage) {}
  default void setElevatorPosition(double positionRotations) {}
  default void resetElevatorPosition(double positionRotations) {}
}
