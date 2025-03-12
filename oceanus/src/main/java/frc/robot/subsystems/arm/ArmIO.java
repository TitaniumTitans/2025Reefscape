package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public class ArmIOInputs {
    public Rotation2d armAngle = new Rotation2d();
    public Rotation2d absoluteArmAngle = new Rotation2d();
  }

  default void updateInputs(ArmIOInputsAutoLogged inputs) {}
  default void setArmPivotVoltage(double voltage) {}
}
