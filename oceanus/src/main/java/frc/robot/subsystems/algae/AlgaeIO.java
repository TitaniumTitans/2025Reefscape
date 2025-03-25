package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
  @AutoLog
  class AlgaeIOInputs {
    public double pivotAngle = 0.0;
    public double pivotVoltage = 0.0;
    public double pivotCurrent = 0.0;
    public double algaeVoltage = 0.0;
    public double algaeCurret = 0.0;
    public boolean hasAlgae = false;
    public boolean limitHit = false;
  }

  public default void updateInputs(AlgaeIOInputsAutoLogged inputs) {}
  public default void setPivotVoltage(double voltage) {}
  public default void setPivotAngle(double degrees) {}
  public default void setAlgaeVoltage() {}
}
