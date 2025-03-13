package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOKraken implements ClimberIO{
  private final TalonFX climber;
  private final PositionVoltage posRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<Current> currentDrawSignal;
  private final StatusSignal<Voltage> appliedVoltageSignal;
  private final NeutralOut stopRequest;
  public ClimberIOKraken() {
    climber = new TalonFX(ClimberConstants.CLIMBER_ID);
    configureDevices();

    positionSignal = climber.getPosition();
    currentDrawSignal = climber.getSupplyCurrent();
    appliedVoltageSignal = climber.getMotorVoltage();

    posRequest = new PositionVoltage(0).withSlot(0)
        .withEnableFOC(climber.getIsProLicensed().getValue());
    stopRequest = new NeutralOut();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        positionSignal,
        currentDrawSignal,
        appliedVoltageSignal);

    climber.setPosition(0.0);
    climber.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(
        positionSignal,
        currentDrawSignal,
        appliedVoltageSignal
    );
    inputs.position = Units.rotationsToDegrees(positionSignal.refresh().getValueAsDouble());
    inputs.currentDraw = currentDrawSignal.refresh().getValueAsDouble();
    inputs.appliedVoltage = appliedVoltageSignal.refresh().getValueAsDouble();
  }
  @Override
  public void setMotorVoltage(double voltage) {
    climber.setVoltage(voltage);
  }
  @Override
  public void setPosititon(double degrees) {
    climber.setControl(posRequest.withPosition(degrees / 360.0));
  }
  @Override
  public void resetPosition() {
    climber.setPosition(0);
  }
  @Override
  public void stop() {
    climber.setControl(stopRequest);
  }
  private void configureDevices() {
    var climberConfig = new TalonFXConfiguration();
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climberConfig.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_GEAR_RATIO;
    climberConfig.Slot0.kP = 0.0;
    climberConfig.Slot0.kI = 0.0;
    climberConfig.Slot0.kD = 0.0;
    climber.getConfigurator().apply(climberConfig);
  }
}
