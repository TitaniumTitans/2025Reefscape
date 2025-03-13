package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double position = 0.0;
    public double currentDraw = 0.0;
    public double appliedVoltage = 0.0;
  }
  default void setMotorVoltage(double voltage) {}
  default void setPosititon(double degrees) {}
  default void updateInputs(ClimberIOInputsAutoLogged inputs) {}
  default void stop() {}
  default void resetPosition() {}
}
