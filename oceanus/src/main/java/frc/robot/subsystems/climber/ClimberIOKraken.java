package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOKraken implements ClimberIO{
  private final TalonFX master;
  private final TalonFX follower;
  private final PositionVoltage posRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<Current> currentDrawSignal;
  private final StatusSignal<Voltage> appliedVoltageSignal;
  private final NeutralOut stopRequest;
  private final Follower followerRequest;

  public ClimberIOKraken() {
    master = new TalonFX(ClimberConstants.CLIMBER_ID);
    follower = new TalonFX(ClimberConstants.FOLLOWER_ID);
    configureDevices();

    positionSignal = master.getPosition();
    currentDrawSignal = master.getSupplyCurrent();
    appliedVoltageSignal = master.getMotorVoltage();

    posRequest = new PositionVoltage(0).withSlot(0)
        .withEnableFOC(master.getIsProLicensed().getValue());
    stopRequest = new NeutralOut();

    followerRequest = new Follower(master.getDeviceID(), true);
    follower.setControl(followerRequest);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        positionSignal,
        currentDrawSignal,
        appliedVoltageSignal);

    master.setPosition(0.0);
    master.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(
        positionSignal,
        currentDrawSignal,
        appliedVoltageSignal
    );
    inputs.position = Rotation2d.fromRotations(positionSignal.getValueAsDouble());
    inputs.currentDraw = currentDrawSignal.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltageSignal.getValueAsDouble();
  }
  @Override
  public void setMotorVoltage(double voltage) {
    master.setVoltage(voltage);
  }
  @Override
  public void setPosititon(double degrees) {
    master.setControl(posRequest.withPosition(degrees / 360.0));
  }
  @Override
  public void resetPosition() {
    master.setPosition(0);
  }
  @Override
  public void stop() {
    master.setControl(stopRequest);
  }
  private void configureDevices() {
    var climberConfig = new TalonFXConfiguration();
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climberConfig.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_GEAR_RATIO;
    climberConfig.Slot0.kP = 0.0;
    climberConfig.Slot0.kI = 0.0;
    climberConfig.Slot0.kD = 0.0;

    climberConfig.CurrentLimits.SupplyCurrentLimit = 70;
    climberConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    climberConfig.CurrentLimits.StatorCurrentLimit = 120;

    master.getConfigurator().apply(climberConfig);
    follower.getConfigurator().apply(climberConfig);
  }
}
