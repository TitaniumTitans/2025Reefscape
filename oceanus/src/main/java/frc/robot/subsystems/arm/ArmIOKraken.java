package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.gos.lib.phoenix6.properties.pid.Phoenix6TalonPidPropertyBuilder;
import com.gos.lib.properties.pid.PidProperty;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class ArmIOKraken implements ArmIO {
  private final TalonFX pivot;
  private final TalonFX rollers;
  private final CANcoder pivotEncoder;

  private final MotionMagicVoltage mmControl;
  private final PidProperty pivotProperty;

  private final StatusSignal<Angle> pivotPositionSignal;
  private final StatusSignal<Angle> pivotAbsolutePositionSignal;
  private final StatusSignal<AngularVelocity> pivotVelocitySignal;
  private final StatusSignal<Voltage> pivotVoltageSignal;
  private final StatusSignal<Current> pivotCurrentSignal;
  private final StatusSignal<Voltage> rollerVoltageSignal;
  private final StatusSignal<Current> rollerCurrentSignal;

  public ArmIOKraken() {
    pivot = new TalonFX(ArmConstants.PIVOT_ID);
    rollers = new TalonFX(ArmConstants.ROLLER_ID);
    pivotEncoder = new CANcoder(ArmConstants.ENCODER_ID);

    pivotProperty = new Phoenix6TalonPidPropertyBuilder(
        "Arm/PID", false, pivot, 0
    )
        .addP(ArmConstants.KP)
        .addI(ArmConstants.KI)
        .addD(ArmConstants.KD)
        .addKG(ArmConstants.KG, GravityTypeValue.Arm_Cosine)
        .build();

    mmControl = new MotionMagicVoltage(0.0);

    pivotPositionSignal = pivot.getPosition();
    pivotAbsolutePositionSignal = pivotEncoder.getAbsolutePosition();
    pivotVelocitySignal = pivot.getVelocity();
    pivotVoltageSignal = pivot.getMotorVoltage();
    pivotCurrentSignal = pivot.getSupplyCurrent();
    rollerVoltageSignal = rollers.getMotorVoltage();
    rollerCurrentSignal = rollers.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        pivotPositionSignal,
        pivotAbsolutePositionSignal,
        pivotVelocitySignal,
        pivotVoltageSignal,
        pivotCurrentSignal,
        rollerVoltageSignal,
        rollerCurrentSignal
        );

    configureDevices();

    pivot.optimizeBusUtilization();
    rollers.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputsAutoLogged inputs) {
    pivotProperty.updateIfChanged();

    inputs.armAngle = Rotation2d.fromRotations(pivotPositionSignal.getValueAsDouble());
    inputs.absoluteArmAngle = Rotation2d.fromRotations(pivotAbsolutePositionSignal.getValueAsDouble());
    inputs.armVoltages = new double[] {
        pivotVoltageSignal.getValueAsDouble(),
        rollerVoltageSignal.getValueAsDouble()
    };
    inputs.armCurrents = new double[] {
        pivotCurrentSignal.getValueAsDouble(),
        rollerCurrentSignal.getValueAsDouble()
    };
  }

  @Override
  public void setArmPivotVoltage(double voltage) {
    pivot.setVoltage(voltage);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollers.setVoltage(voltage);
  }

  @Override
  public void setArmPivotAngle(Rotation2d angle) {
    Logger.recordOutput("Arm/Angle Setpoint", angle);
    pivot.setControl(mmControl.withPosition(angle.getRotations()));
  }

  private void configureDevices() {
    var motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.Feedback.SensorToMechanismRatio = ArmConstants.PIVOT_GEAR_RATIO;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRotations(30);
    motorConfig.MotionMagic.MotionMagicAcceleration = Units.degreesToRotations(30);

    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;

    pivot.getConfigurator().apply(motorConfig);

    var encoderConfig = new CANcoderConfiguration();

    encoderConfig.MagnetSensor.MagnetOffset = 0.0;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    pivotEncoder.getConfigurator().apply(encoderConfig);

    pivot.setPosition(
        pivotAbsolutePositionSignal.getValueAsDouble() - ArmConstants.PIVOT_ENCODER_OFFSET.getRotations()
    );
  }
}
