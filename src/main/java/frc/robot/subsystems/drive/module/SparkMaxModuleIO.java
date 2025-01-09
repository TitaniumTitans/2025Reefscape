package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;

public class SparkMaxModuleIO implements ModuleIO {
  private final SparkMax m_drive;
  private final SparkMax m_steer;
  private final CANcoder m_encoder;
  SparkClosedLoopController m_driveController;
  SparkClosedLoopController m_steerController;
  public SparkMaxModuleIO(SparkMax drive, SparkMax steer) {
    m_drive = drive;
    m_steer = steer;
    m_driveController = m_drive.getClosedLoopController();
    m_steerController = m_steer.getClosedLoopController();
    m_encoder = new CANcoder(1);

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(30)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(driveEncoderPositionFactor)
        .velocityConversionFactor(driveEncoderVelocityFactor);
    driveConfig
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pidf(
            driveKp, 0.0,
            driveKd, 0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
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
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    steerConfig
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pidf(turnKp, 0.0, turnKd, 0.0);
    steerConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    m_steer.configure(steerConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    m_driveController.setReference(setPoint, SparkBase.ControlType.kPosition);
    driveConfig.closedLoop
        .p(0)
        .i(0)
        .d(0);
  }

  @Override
  public void updateInputs(ModuleIOInputsAutoLogged inputs) {
    ModuleIO.super.updateInputs(inputs);
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
    m_driveController
  }

  @Override
  public void setSteerPosition(Rotation2d rotation) {
    driveConfig.
  }
}
