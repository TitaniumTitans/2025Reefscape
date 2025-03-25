package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public Rotation2d position = new Rotation2d();
    public double currentDraw = 0.0;
    public double appliedVoltage = 0.0;
  }
  default void setMotorVoltage(double voltage) {}
  default void setPosititon(double degrees) {}
  default void updateInputs(ClimberIOInputsAutoLogged inputs) {}
  default void stop() {}
  default void resetPosition() {}
}
