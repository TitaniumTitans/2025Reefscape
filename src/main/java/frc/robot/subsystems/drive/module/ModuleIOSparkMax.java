package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;

import java.util.function.DoubleSupplier;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;

public class ModuleIOSparkMax implements ModuleIO {
  private final SparkMax drive;
  private final SparkMax steer;
  private final CANcoder encoder;
  RelativeEncoder driveEncoder;
  RelativeEncoder steerEncoder;
  SparkClosedLoopController m_driveController;
  SparkClosedLoopController m_steerController;

  DriveConstants.ModuleConstants config;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  public ModuleIOSparkMax(DriveConstants.ModuleConstants config) {
    this.config = config;
    drive = new SparkMax(config.driveId(), SparkLowLevel.MotorType.kBrushless);
    steer = new SparkMax(config.steerId(), SparkLowLevel.MotorType.kBrushless);
    encoder = new CANcoder(config.encoderId());

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(30)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(DriveConstants.DRIVE_GEAR_RATIO)
        .velocityConversionFactor(DriveConstants.DRIVE_GEAR_RATIO);
    driveConfig
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pidf(
            0.5, 0.0,
            0.0, 0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.ODOMETRY_FREQUENCY))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    drive.configure(driveConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    SparkMaxConfig steerConfig = new SparkMaxConfig();
    steerConfig
        .inverted(true)
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(30)
        .voltageCompensation(12.0);
    steerConfig
        .encoder
        .inverted(true)
        .positionConversionFactor(DriveConstants.STEER_GEAR_RATIO)
        .velocityConversionFactor(DriveConstants.STEER_GEAR_RATIO);
    steerConfig
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0.0, 1.0)
        .pidf(0.5, 0.0, 0.0, 0.0);
    steerConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.ODOMETRY_FREQUENCY))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    steer.configure(steerConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = config.encoderOffset().getRotations();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(encoderConfig);

    m_driveController = drive.getClosedLoopController();
    m_steerController = steer.getClosedLoopController();

    driveEncoder = drive.getEncoder();
    steerEncoder = steer.getEncoder();

    steerEncoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void updateInputs(ModuleIOInputsAutoLogged inputs) {
    sparkStickyFault = false;
    ifOk(drive, driveEncoder::getPosition, (value) -> inputs.drivePositionRads = Units.rotationsToRadians(value));
    ifOk(drive, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(value));
    ifOk(
        drive,
        new DoubleSupplier[] {drive::getAppliedOutput, drive::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(drive, drive::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        steer,
        steerEncoder::getPosition,
        (value) -> inputs.steerPosition = Rotation2d.fromRotations(value).minus(config.encoderOffset()));
    ifOk(steer, steerEncoder::getVelocity, (value) -> inputs.steerVelocityRadsPerSec = Units.rotationsToRadians(value));
    ifOk(
        steer,
        new DoubleSupplier[] {steer::getAppliedOutput, steer::getBusVoltage},
        (values) -> inputs.steerAppliedVolts = values[0] * values[1]);
    ifOk(steer, steer::getOutputCurrent, (value) -> inputs.steerCurrentAmps = value);
    inputs.steerConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    inputs.steerAbsolutePosition = Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
  }


  @Override
  public void setDriveOpenLoop(double volts) {
    drive.setVoltage(volts);
  }

  @Override
  public void setSteerOpenLoop(double volts) {
    steer.setVoltage(volts);
  }

  @Override
  public void setDriveVelocity(double radsPerSec) {
    m_driveController.setReference(Units.radiansToRotations(radsPerSec), SparkBase.ControlType.kVelocity);
  }

  @Override
  public void setSteerPosition(Rotation2d rotation) {
    m_steerController.setReference(rotation.plus(config.encoderOffset()).getRotations(), SparkBase.ControlType.kPosition);
  }
}
