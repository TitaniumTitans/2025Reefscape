package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class ArmIOKraken implements ArmIO {
  private final TalonFX pivot;
  private final CANcoder pivotEncoder;

  public ArmIOKraken() {
    pivot = new TalonFX(16);
    pivotEncoder = new CANcoder(17);
  }

  @Override
  public void updateInputs(ArmIOInputsAutoLogged inputs) {
    inputs.armAngle = Rotation2d.fromRotations(pivot.getPosition().getValueAsDouble());
    inputs.absoluteArmAngle = Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void setArmPivotVoltage(double voltage) {
    pivot.setVoltage(voltage);
  }

  private void configureDevices() {
    var motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.Feedback.SensorToMechanismRatio = ArmConstants.PIVOT_GEAR_RATIO;

    pivot.getConfigurator().apply(motorConfig);

    var encoderConfig = new CANcoderConfiguration();

    encoderConfig.MagnetSensor.MagnetOffset = 0.0;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    pivotEncoder.getConfigurator().apply(encoderConfig);
  }
}
