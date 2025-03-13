package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;

public class CoralIOTalon implements CoralIO {
  private final TalonFX pivot;
  private final TalonFX hopper;
  private final TalonFX outerCoral;
  private final TalonFX innerCoral;

  private final MotionMagicVoltage mmControl;

  public CoralIOTalon() {
    pivot = new TalonFX(CoralConstants.PIVOT_ID);
    hopper = new TalonFX(CoralConstants.HOPPER_ID);
    outerCoral = new TalonFX(CoralConstants.OUTER_ID);
    innerCoral = new TalonFX(CoralConstants.INNER_ID);

    mmControl = new MotionMagicVoltage(0.0);
  }

  @Override
  public void updateInputs(CoralIOInputsAutoLogged inputs) {
    inputs.coralAppliedVoltage = new double[] {
        pivot.getMotorVoltage().getValueAsDouble(),
        hopper.getMotorVoltage().getValueAsDouble(),
        outerCoral.getMotorVoltage().getValueAsDouble(),
        innerCoral.getMotorVoltage().getValueAsDouble()
    };
    inputs.coralCurrentDraw = new double[] {
        pivot.getSupplyCurrent().getValueAsDouble(),
        hopper.getSupplyCurrent().getValueAsDouble(),
        outerCoral.getSupplyCurrent().getValueAsDouble(),
        innerCoral.getSupplyCurrent().getValueAsDouble()
    };
    inputs.hasCoral = false;
    inputs.coralPivotAngle = Rotation2d.fromRotations(pivot.getPosition().getValueAsDouble());
  }

  @Override
  public void setPivotAngle(double appliedVolts) {
    CoralIO.super.setPivotAngle(appliedVolts);
  }

  @Override
  public void setPivotVoltage(double appliedVolts) {
    pivot.setVoltage(appliedVolts);
  }

  @Override
  public void setHopperVoltage(double appliedVolts) {
    hopper.setVoltage(appliedVolts);
  }

  @Override
  public void setCoralVoltage(double outerVolt, double innerVolt) {
    outerCoral.setVoltage(outerVolt);
    innerCoral.setVoltage(innerVolt);
  }

  public void configureDevices() {
    var motorConfig = new TalonFXConfiguration();

    motorConfig.Feedback.SensorToMechanismRatio = CoralConstants.PIVOT_GEAR_RATIO;
  }
}
