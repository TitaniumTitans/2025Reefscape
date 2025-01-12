package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOProto implements IntakeIO {
  private final TalonFX intake;

  public IntakeIOProto() {
    intake = new TalonFX(17);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intake.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeIOInputsAutoLogged inputs) {
    inputs.intakeAppliedVolts = intake.getMotorVoltage().getValueAsDouble();
    inputs.intakeCurrentDraw = intake.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setMotorVoltageIntake(double voltage) {
    intake.setVoltage(voltage);
  }
}
