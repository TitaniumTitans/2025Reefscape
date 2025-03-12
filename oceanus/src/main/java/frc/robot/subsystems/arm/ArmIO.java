package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public class ArmIOInputs {
    public Rotation2d armAngle = new Rotation2d();
    public Rotation2d absoluteArmAngle = new Rotation2d();
    public double[] armVoltages = new double[] {0.0, 0.0};
    public double [] armCurrents = new double[] {0.0, 0.0};
  }

  default void updateInputs(ArmIOInputsAutoLogged inputs) {}
  default void setArmPivotVoltage(double voltage) {}
  default void setArmPivotAngle(Rotation2d angle) {}
  default void setRollerVoltage(double voltage) {}
}
