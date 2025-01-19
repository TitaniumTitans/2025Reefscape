package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.PhoenixOdometryThread;

import java.util.Queue;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder encoder;
  private final DriveConstants.ModuleConstants config;

  private final VelocityVoltage driveRequest;
  private final PositionVoltage steerRequest;

  // drive motor signals
  private final StatusSignal<Angle> drivePositionSignal;
  private final StatusSignal<AngularVelocity> driveVelocitySignal;
  private final StatusSignal<Voltage> driveVoltageSignal;
  private final StatusSignal<Current> driveCurrentSignal;

  // steer motor signals
  private final StatusSignal<Angle> steerPositionSignal;
  private final StatusSignal<AngularVelocity> steerVelocitySignal;
  private final StatusSignal<Voltage> steerVoltageSignal;
  private final StatusSignal<Current> steerCurrentSignal;

  // odometry signals
  private final Queue<Double> odometryTimestampQueue;
  private final Queue<Double> odometrySteerPositionQueue;
  private final Queue<Double> odometryDrivePositionQueue;

  public ModuleIOTalonFX(DriveConstants.ModuleConstants config) {
    this.config = config;

    driveMotor = new TalonFX(config.driveId());
    steerMotor = new TalonFX(config.steerId());
    encoder = new CANcoder(config.encoderId());

    configureDevices();

    driveRequest = new VelocityVoltage(0.0)
        .withEnableFOC(true)
        .withSlot(0);
    steerRequest = new PositionVoltage(0.0)
        .withEnableFOC(true)
        .withSlot(0);

    drivePositionSignal = driveMotor.getPosition();
    driveVelocitySignal = driveMotor.getVelocity();
    driveVoltageSignal = driveMotor.getMotorVoltage();
    driveCurrentSignal = driveMotor.getStatorCurrent();

    steerPositionSignal = steerMotor.getPosition();
    steerVelocitySignal = steerMotor.getVelocity();
    steerVoltageSignal = steerMotor.getMotorVoltage();
    steerCurrentSignal = steerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocitySignal,
        driveVelocitySignal,
        driveCurrentSignal,
        steerVelocitySignal,
        steerVoltageSignal,
        steerCurrentSignal
    );

    odometryTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    odometrySteerPositionQueue = PhoenixOdometryThread.getInstance()
        .registerSignal(steerMotor.getPosition());
    odometryDrivePositionQueue = PhoenixOdometryThread.getInstance()
        .registerSignal(driveMotor.getPosition());

    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.ODOMETRY_FREQUENCY, drivePositionSignal, steerPositionSignal
    );

    driveMotor.optimizeBusUtilization();
    steerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(
        drivePositionSignal,
        driveVelocitySignal,
        driveVelocitySignal,
        driveCurrentSignal,
        steerPositionSignal,
        steerVelocitySignal,
        steerVoltageSignal,
        steerCurrentSignal
    );

    inputs.driveConnected = driveMotor.isConnected();
    inputs.drivePositionRads = Units.rotationsToRadians(drivePositionSignal.getValueAsDouble());
    inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(driveVelocitySignal.getValueAsDouble());
    inputs.driveAppliedVolts = driveVoltageSignal.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrentSignal.getValueAsDouble();

    inputs.steerConnected = steerMotor.isConnected();
    inputs.steerPosition = Rotation2d.fromRotations(
        Units.rotationsToRadians(drivePositionSignal.getValueAsDouble()));
    inputs.steerVelocityRadsPerSec = Units.rotationsToRadians(steerVelocitySignal.getValueAsDouble());
    inputs.steerAppliedVolts = steerVoltageSignal.getValueAsDouble();
    inputs.steerCurrentAmps = steerCurrentSignal.getValueAsDouble();

    inputs.odometryTimestamps =
        odometryTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRads =
        odometryDrivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
    inputs.odometrySteerPositions =
        odometrySteerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveMotor.setVoltage(volts);
  }

  @Override
  public void setSteerOpenLoop(double volts) {
    steerMotor.setVoltage(volts);
  }

  @Override
  public void setDriveVelocity(double radsPerSec) {
    driveMotor.setControl(driveRequest.withVelocity(RadiansPerSecond.of(radsPerSec)));
  }

  @Override
  public void setSteerPosition(Rotation2d rotation) {
    steerMotor.setControl(steerRequest.withPosition(rotation.getRotations()));
  }

  private void configureDevices() {
    // Drive motor
    var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.Feedback.SensorToMechanismRatio = DriveConstants.DRIVE_GEAR_RATIO;

    motorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withSupplyCurrentLowerTime(0.1)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(80);

    motorConfig.Slot0.withKP(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0);

    driveMotor.getConfigurator().apply(motorConfig);

    motorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    motorConfig.CurrentLimits.withStatorCurrentLimit(60);
    motorConfig.Slot0.withKP(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0);

    steerMotor.getConfigurator().apply(motorConfig);

    var encoderConfig = new CANcoderConfiguration();
    encoder.getConfigurator().apply(encoderConfig);
  }
}
