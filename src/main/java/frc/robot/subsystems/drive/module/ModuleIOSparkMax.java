package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.ModuleConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;

public class ModuleIOSparkMax implements ModuleIO {
  private final SparkMax m_drive;
  private final SparkMax m_steer;
  private final CANcoder m_encoder;
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
    m_drive = new SparkMax(config.driveId(), SparkLowLevel.MotorType.kBrushless);
    m_steer = new SparkMax(config.steerId(), SparkLowLevel.MotorType.kBrushless);
    m_encoder = new CANcoder(config.encoderId());

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
    m_drive.configure(driveConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    SparkMaxConfig steerConfig = new SparkMaxConfig();
    steerConfig
        .inverted(true)
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(30)
        .voltageCompensation(12.0);
    steerConfig
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor(DriveConstants.STEER_GEAR_RATIO)
        .velocityConversionFactor(DriveConstants.STEER_GEAR_RATIO)
        .averageDepth(2);
    steerConfig
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
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
    m_steer.configure(steerConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    m_driveController = m_drive.getClosedLoopController();
    m_steerController = m_steer.getClosedLoopController();

    driveEncoder = m_drive.getEncoder();
    steerEncoder = m_steer.getEncoder();
  }

  @Override
  public void updateInputs(ModuleIOInputsAutoLogged inputs) {
    sparkStickyFault = false;
    ifOk(m_drive, driveEncoder::getPosition, (value) -> inputs.drivePositionRads = Units.rotationsToRadians(value));
    ifOk(m_drive, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(value));
    ifOk(
        m_drive,
        new DoubleSupplier[] {m_drive::getAppliedOutput, m_drive::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(m_drive, m_drive::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        m_steer,
        steerEncoder::getPosition,
        (value) -> inputs.steerPosition = Rotation2d.fromRotations(value).minus(config.encoderOffset()));
    ifOk(m_steer, steerEncoder::getVelocity, (value) -> inputs.steerVelocityRadsPerSec = Units.rotationsToRadians(value));
    ifOk(
        m_steer,
        new DoubleSupplier[] {m_steer::getAppliedOutput, m_steer::getBusVoltage},
        (values) -> inputs.steerAppliedVolts = values[0] * values[1]);
    ifOk(m_steer, m_steer::getOutputCurrent, (value) -> inputs.steerCurrentAmps = value);
    inputs.steerConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    inputs.steerAbsolutePosition = Rotation2d.fromRotations(m_encoder.getAbsolutePosition().getValueAsDouble());
  }


  @Override
  public void setDriveOpenLoop(double volts) {
    m_drive.setVoltage(volts);
  }

  @Override
  public void setSteerOpenLoop(double volts) {
    m_steer.setVoltage(volts);
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
