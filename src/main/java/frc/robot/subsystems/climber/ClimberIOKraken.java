package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class ClimberIOKraken implements ClimberIO{
    private final TalonFX climber;
    private final PositionVoltage posRequest;
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Current> currentDrawSignal;
    private final StatusSignal<Double> setpointSignal;
    private final StatusSignal<Double> appliedOutputSignal;
    private final NeutralOut stopRequest;
    public ClimberIOKraken() {
        climber = new TalonFX(ClimberConstants.CLIMBER_ID);
        TalonFXConfiguration climberConfig = new TalonFXConfiguration();
        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        climberConfig.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_GEAR_RATIO;
        climberConfig.Slot0.kP = 0.0;
        climberConfig.Slot0.kI = 0;
        climberConfig.Slot0.kD = 0.0;
        climber.getConfigurator().apply(climberConfig);

        positionSignal = climber.getPosition();
        currentDrawSignal = climber.getSupplyCurrent();
        setpointSignal = climber.getClosedLoopReference();
        appliedOutputSignal = climber.getDutyCycle();

        posRequest = new PositionVoltage(0).withSlot(0)
                .withEnableFOC(climber.getIsProLicensed().getValue());
        stopRequest = new NeutralOut();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
                positionSignal,
                currentDrawSignal,
                setpointSignal,
                appliedOutputSignal);

        climber.optimizeBusUtilization();
    }
    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(
                positionSignal,
                currentDrawSignal,
                setpointSignal,
                appliedOutputSignal
        );
        inputs.position = positionSignal.refresh().getValueAsDouble();
        inputs.currentDraw = currentDrawSignal.refresh().getValueAsDouble();
        inputs.setpoint = setpointSignal.refresh().getValueAsDouble();
        inputs.appliedOutput = appliedOutputSignal.refresh().getValueAsDouble();

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
}