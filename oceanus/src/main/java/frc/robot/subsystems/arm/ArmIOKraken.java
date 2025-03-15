package frc.robot.subsystems.arm;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.gos.lib.phoenix6.properties.pid.Phoenix6TalonPidPropertyBuilder;
import com.gos.lib.properties.pid.PidProperty;
import edu.wpi.first.math.filter.Debouncer;
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
  private final LaserCan laserCan;

  private final Debouncer debouncer = new Debouncer(0.25);

  private final MotionMagicVoltage mmControl;

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
    laserCan = new LaserCan(ArmConstants.LASER_CAN_ID);

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
    BaseStatusSignal.refreshAll(
        pivotPositionSignal,
        pivotAbsolutePositionSignal,
        pivotVelocitySignal,
        pivotVoltageSignal,
        pivotCurrentSignal,
        rollerVoltageSignal,
        rollerCurrentSignal);

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

    var measurement = laserCan.getMeasurement();
    if (measurement != null) {
      inputs.hasCoral = debouncer.calculate(measurement.distance_mm < 30);
    } else {
      inputs.hasCoral = false;
    }
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
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorConfig.Feedback.RotorToSensorRatio = ArmConstants.PIVOT_GEAR_RATIO;
    motorConfig.Feedback.SensorToMechanismRatio = 1.0;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRotations(180);
    motorConfig.MotionMagic.MotionMagicAcceleration = Units.degreesToRotations(180);

    motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    motorConfig.CurrentLimits.StatorCurrentLimit = 100;

    motorConfig.Slot0.kP = ArmConstants.KP;
    motorConfig.Slot0.kI = ArmConstants.KI;
    motorConfig.Slot0.kD = ArmConstants.KD;
    motorConfig.Slot0.kG = ArmConstants.KG;
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;


    pivot.getConfigurator().apply(motorConfig);

    var encoderConfig = new CANcoderConfiguration();

    encoderConfig.MagnetSensor.MagnetOffset = ArmConstants.PIVOT_ENCODER_OFFSET.unaryMinus().getRotations();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    pivotEncoder.getConfigurator().apply(encoderConfig);

    pivot.setPosition(pivotAbsolutePositionSignal.refresh().getValueAsDouble());
  }
}
