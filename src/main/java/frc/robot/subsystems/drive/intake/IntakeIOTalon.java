package frc.robot.subsystems.drive.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

//Current limiting config - 20 amps on both supply and stater
public class IntakeIOTalon implements IntakeIO {
    private final TalonFX intake;
    private final TalonFX pivot;
    private final StatusSignal<Angle> pivotPositionSignal;
    private final StatusSignal<Voltage> intakeVoltageSignal;
    private final StatusSignal<Voltage> pivotVoltageSignal;
    private final StatusSignal<Current> intakeDrawSignal;
    private final StatusSignal<Current> pivotDrawSignal;
    private final StatusSignal<Voltage> intakeAppliedVoltsSignal;
    private final StatusSignal<Voltage> pivotAppliedVoltsSignal;
    private final StatusSignal<Temperature> intakeTemperatureSignal;
    private final StatusSignal<Temperature> pivotTemperatureSignal;
    private final NeutralOut stopRequest;


    public IntakeIOTalon() {
        intake = new TalonFX(IntakeConstants.INTAKE_ID);
        pivot = new TalonFX(IntakeConstants.PIVOT_ID);

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 20;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.StatorCurrentLimit = 20;
        intake.getConfigurator().apply(intakeConfig);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 20;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = 20;
        pivot.getConfigurator().apply(pivotConfig);

        var intakeConfigs = new Slot0Configs();
        intakeConfigs.kP = 2.4;
        intakeConfigs.kI = 0;
        intakeConfigs.kD = 0.1;
        intake.getConfigurator().apply(intakeConfigs);
        final PositionVoltage intakeRequest = new PositionVoltage(0).withSlot(0);
        intake.setControl(intakeRequest.withPosition(10));

        var pivotConfigs = new Slot0Configs();
        pivotConfigs.kP = 2.4;
        pivotConfigs.kI = 0;
        pivotConfigs.kD = 0.1;
        pivot.getConfigurator().apply(pivotConfigs);
        final PositionVoltage pivotRequest = new PositionVoltage(0).withSlot(0);
        pivot.setControl(pivotRequest.withPosition(10));

        stopRequest = new NeutralOut();

        pivotPositionSignal = pivot.getPosition();
        intakeVoltageSignal = intake.getMotorVoltage();
        pivotVoltageSignal = pivot.getMotorVoltage();
        intakeDrawSignal = intake.getSupplyCurrent();
        pivotDrawSignal = pivot.getSupplyCurrent();
        intakeAppliedVoltsSignal = intake.getSupplyVoltage();
        pivotAppliedVoltsSignal = pivot.getSupplyVoltage();
        intakeTemperatureSignal = intake.getDeviceTemp();
        pivotTemperatureSignal = pivot.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
                pivotPositionSignal,
                intakeVoltageSignal,
                pivotVoltageSignal,
                intakeDrawSignal,
                pivotDrawSignal,
                intakeAppliedVoltsSignal,
                pivotAppliedVoltsSignal,
                intakeTemperatureSignal,
                pivotTemperatureSignal);


        intake.optimizeBusUtilization();
        pivot.optimizeBusUtilization();
    }

        @Override
        public void updateInputs (IntakeIOInputsAutoLogged inputs){
            BaseStatusSignal.refreshAll(
                    intakeVoltageSignal,
                    pivotVoltageSignal,
                    intakeDrawSignal,
                    pivotDrawSignal,
                    intakeAppliedVoltsSignal,
                    pivotAppliedVoltsSignal,
                    intakeTemperatureSignal,
                    pivotTemperatureSignal
            );
            inputs.intakeVoltage = intakeVoltageSignal.refresh().getValueAsDouble();
            inputs.pivotVoltage = pivotVoltageSignal.refresh().getValueAsDouble();
            inputs.intakeDraw = intakeDrawSignal.refresh().getValueAsDouble();
            inputs.pivotDraw = pivotDrawSignal.refresh().getValueAsDouble();
            inputs.intakeAppliedVolts = intakeAppliedVoltsSignal.refresh().getValueAsDouble();
            inputs.pivotAppliedVolts = pivotAppliedVoltsSignal.refresh().getValueAsDouble();
            inputs.intakeTemperature = intakeTemperatureSignal.refresh().getValueAsDouble();
            inputs.pivotTemperature = pivotTemperatureSignal.refresh().getValueAsDouble();
        }

    @Override
    public void setMotorVoltageIntake(double voltage) {
        intake.setVoltage(voltage);
    }
    @Override
    public void setMotorVoltagePivot(double voltage) {
        pivot.setVoltage(voltage);
    }
    @Override
    public void stop() {
        intake.setControl(stopRequest);
    }

}