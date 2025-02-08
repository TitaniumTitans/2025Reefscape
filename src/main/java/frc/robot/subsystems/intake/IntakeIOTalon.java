package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

//Current limiting config - 20 amps on both supply and stater
public class IntakeIOTalon implements IntakeIO {
    private final TalonFX intake;
    private final TalonFX pivot;
    private final StatusSignal<Angle> pivotPositionSignal;
    private final StatusSignal<Voltage> intakeVoltageSignal;
    private final StatusSignal<Voltage> pivotVoltageSignal;
    private final StatusSignal<Current> intakeDrawSignal;
    private final StatusSignal<Current> pivotDrawSignal;
    private final StatusSignal<Temperature> intakeTemperatureSignal;
    private final StatusSignal<Temperature> pivotTemperatureSignal;
    private final StatusSignal<AngularVelocity> intakeVelocitySignal;
    private final StatusSignal<AngularVelocity> pivotVelocitySignal;
    private final NeutralOut stopRequest;

    private final MotionMagicVoltage mmPositionRequest;

    public IntakeIOTalon() {
        intake = new TalonFX(IntakeConstants.INTAKE_ID);
        pivot = new TalonFX(IntakeConstants.PIVOT_ID);

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 60;
        intakeConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.StatorCurrentLimit = 100;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intake.getConfigurator().apply(intakeConfig);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 70;
        intakeConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;

        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = 80;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_GEAR_RATIO;
        pivotConfig.Slot0.kP = IntakeConstants.PIVOT_KP;
        pivotConfig.Slot0.kI = IntakeConstants.PIVOT_KI;
        pivotConfig.Slot0.kD = IntakeConstants.PIVOT_KD;
        pivotConfig.Slot0.kG = IntakeConstants.PIVOT_KG;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 25;
        pivotConfig.MotionMagic.MotionMagicAcceleration = 20;
        pivot.getConfigurator().apply(pivotConfig);

        var intakeConfigs = new Slot0Configs();
        intakeConfigs.kP = 0.0;
        intakeConfigs.kI = 0;
        intakeConfigs.kD = 0.0;
        intake.getConfigurator().apply(intakeConfigs);
        final PositionVoltage intakeRequest = new PositionVoltage(0).withSlot(0);
        intake.setControl(intakeRequest.withPosition(10));

        mmPositionRequest = new MotionMagicVoltage(0.0)
            .withSlot(0);
        stopRequest = new NeutralOut();

        pivotPositionSignal = pivot.getPosition();
        intakeVoltageSignal = intake.getMotorVoltage();
        pivotVoltageSignal = pivot.getMotorVoltage();
        intakeDrawSignal = intake.getSupplyCurrent();
        pivotDrawSignal = pivot.getSupplyCurrent();
        intakeTemperatureSignal = intake.getDeviceTemp();
        pivotTemperatureSignal = pivot.getDeviceTemp();
        intakeVelocitySignal = intake.getVelocity();
        pivotVelocitySignal = pivot.getVelocity();

        zeroPivot();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
                pivotPositionSignal,
                intakeVoltageSignal,
                pivotVoltageSignal,
                intakeDrawSignal,
                pivotDrawSignal,
                intakeTemperatureSignal,
                pivotTemperatureSignal,
                intakeVelocitySignal,
                pivotVelocitySignal);


        intake.optimizeBusUtilization();
        pivot.optimizeBusUtilization();
    }

        @Override
        public void updateInputs (IntakeIOInputsAutoLogged inputs){
            BaseStatusSignal.refreshAll(
                    pivotPositionSignal,
                    intakeVoltageSignal,
                    pivotVoltageSignal,
                    intakeDrawSignal,
                    pivotDrawSignal,
                    intakeTemperatureSignal,
                    pivotTemperatureSignal,
                    intakeVelocitySignal,
                    pivotVelocitySignal
            );

            inputs.pivotPosititon = Rotation2d.fromRotations(pivotPositionSignal.getValueAsDouble());
            inputs.intakeVoltage = intakeVoltageSignal.refresh().getValueAsDouble();
            inputs.pivotVoltage = pivotVoltageSignal.refresh().getValueAsDouble();
            inputs.intakeCurrentDraw = intakeDrawSignal.refresh().getValueAsDouble();
            inputs.pivotCurrentDraw = pivotDrawSignal.refresh().getValueAsDouble();
            inputs.intakeTemperature = intakeTemperatureSignal.refresh().getValueAsDouble();
            inputs.pivotTemperature = pivotTemperatureSignal.refresh().getValueAsDouble();
            inputs.intakeVelocity = intakeVelocitySignal.refresh().getValueAsDouble();
            inputs.pivotVelocity = pivotVelocitySignal.refresh().getValueAsDouble();
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
    public void setPivotAngle(double angleDegrees) {
        pivot.setControl(mmPositionRequest.withPosition(Units.degreesToRotations(angleDegrees)));
    }

    @Override
    public void stop() {
        intake.setControl(stopRequest);
    }

    @Override
    public void zeroPivot() {
        pivot.setPosition((5.625 + 90) / 360.0);
    }
}