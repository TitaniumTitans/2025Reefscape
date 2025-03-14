package frc.robot.subsystems.coral;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
  @AutoLog
  class CoralIOInputs {
    public Rotation2d coralPivotAngle = new Rotation2d();
    public boolean hasCoral = false;
    public double[] coralAppliedVoltage = new double[] {0.0, 0.0, 0.0, 0.0};
    public double[] coralCurrentDraw = new double[] {0.0, 0.0, 0.0, 0.0};
  }

  default void updateInputs(CoralIOInputsAutoLogged inputs) {}
  default void setPivotVoltage(double appliedVolts) {}
  default void setPivotAngle(double angleDegrees) {}
  default void setHopperVoltage(double appliedVolts) {}
  default void setCoralVoltage(double outerVolt, double innerVolt) {}
}
