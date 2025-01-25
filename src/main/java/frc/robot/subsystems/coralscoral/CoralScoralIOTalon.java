package frc.robot.subsystems.coralscoral;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.CoralScoralIO;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import frc.robot.subsystems.CoralScoralIOInputsAutoLogged;

//inverted and neutral mode coast
public class CoralScoralIOTalon implements CoralScoralIO {
    private final TalonFX scorer;
    private final TalonFX masterPivot;
    private final TalonFX followerPivot;
    private final StatusSignal<Angle> pivotPositionSignal;
    private final StatusSignal<AngularVelocity> pivotVelocitySignal;
    private final StatusSignal<AngularVelocity> scorerVelocity;
    private final StatusSignal<Voltage> scorerVoltageSignal;
    private final StatusSignal<Voltage> masterPivotVoltageSignal;
    private final StatusSignal<Voltage> followerPivotVoltageSignal;
    private final StatusSignal<Current> scorerCurrentDrawSignal;
    private final StatusSignal<Current> masterPivotCurrentDrawSignal;
    private final StatusSignal<Current> followerPivotCurrentDrawSignal;
    private final StatusSignal<Temperature> scorerTemperatureSignal;
    private final StatusSignal<Temperature> masterPivotTemperatureSignal;
    private final StatusSignal<Temperature> followerPivotTemperatureSignal;
    private final Follower pivotFollowerRequest;
    private final NeutralOut stopRequest;

    public CoralScoralIOTalon() {
        scorer = new TalonFX(CoralScoralConstants.SCORER_ID);
        masterPivot = new TalonFX(CoralScoralConstants.MASTER_PIVOT_ID);
        followerPivot = new TalonFX(CoralScoralConstants.FOLLOWER_PIVOT_ID);

        TalonFXConfiguration scorerConfig = new TalonFXConfiguration();
        scorerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        scorerConfig.CurrentLimits.SupplyCurrentLimit = 20;
        scorerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        scorerConfig.CurrentLimits.StatorCurrentLimit = 20;
        scorer.getConfigurator().apply(scorerConfig);

        TalonFXConfiguration masterPivotConfig = new TalonFXConfiguration();
        masterPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        masterPivotConfig.CurrentLimits.StatorCurrentLimit = 20;
        masterPivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        masterPivotConfig.CurrentLimits.StatorCurrentLimit = 20;
        masterPivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        masterPivot.getConfigurator().apply(masterPivotConfig);

        TalonFXConfiguration followerPivotConfig = new TalonFXConfiguration();
        followerPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        followerPivotConfig.CurrentLimits.StatorCurrentLimit = 20;
        followerPivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        followerPivotConfig.CurrentLimits.StatorCurrentLimit = 20;
        followerPivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followerPivot.getConfigurator().apply(followerPivotConfig);

        pivotFollowerRequest = new Follower(CoralScoralConstants.MASTER_PIVOT_ID, false);

        var scorerConfigs = new Slot0Configs();
        scorerConfigs.kP = 0.0;
        scorerConfigs.kI = 0.0;
        scorerConfigs.kD = 0.0;
        scorer.getConfigurator().apply(scorerConfigs);
        final PositionVoltage scorerRequest = new PositionVoltage(0).withSlot(0);
        scorer.setControl(scorerRequest.withPosition(10));

        var pivotConfigs = new Slot0Configs();
        pivotConfigs.kP = 0.0;
        pivotConfigs.kI = 0.0;
        pivotConfigs.kD = 0.0;
        masterPivot.getConfigurator().apply(pivotConfigs);
        final PositionVoltage pivotRequest = new PositionVoltage(0).withSlot(0);
        masterPivot.setControl(pivotRequest.withPosition(10));

        stopRequest = new NeutralOut();

        pivotPositionSignal = masterPivot.getPosition();
        pivotVelocitySignal = masterPivot.getVelocity();
        scorerVelocity = scorer.getVelocity();
        scorerVoltageSignal = scorer.getMotorVoltage();
        masterPivotVoltageSignal = masterPivot.getMotorVoltage();
        followerPivotVoltageSignal = followerPivot.getMotorVoltage();
        scorerCurrentDrawSignal = scorer.getSupplyCurrent();
        masterPivotCurrentDrawSignal = masterPivot.getSupplyCurrent();
        followerPivotCurrentDrawSignal = followerPivot.getSupplyCurrent();
        scorerTemperatureSignal = scorer.getDeviceTemp();
        masterPivotTemperatureSignal = masterPivot.getDeviceTemp();
        followerPivotTemperatureSignal = followerPivot.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
                pivotPositionSignal,
                pivotVelocitySignal,
                scorerVelocity,
                scorerVoltageSignal,
                masterPivotVoltageSignal,
                followerPivotVoltageSignal,
                scorerCurrentDrawSignal,
                masterPivotCurrentDrawSignal,
                followerPivotCurrentDrawSignal,
                scorerTemperatureSignal,
                masterPivotTemperatureSignal,
                followerPivotTemperatureSignal);

        scorer.optimizeBusUtilization();
        masterPivot.optimizeBusUtilization();
        followerPivot.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(CoralScoralIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(
                pivotPositionSignal,
                pivotVelocitySignal,
                scorerVelocity,
                scorerVoltageSignal,
                masterPivotVoltageSignal,
                followerPivotVoltageSignal,
                scorerCurrentDrawSignal,
                masterPivotCurrentDrawSignal,
                followerPivotCurrentDrawSignal,
                scorerTemperatureSignal,
                masterPivotTemperatureSignal,
                followerPivotTemperatureSignal
        );
        inputs.pivotPosition = Rotation2d.fromRotations(pivotPositionSignal.refresh().getValueAsDouble());
        inputs.pivotVelocity = pivotVelocitySignal.refresh().getValueAsDouble();
        inputs.scorerVelocity = scorerVelocity.refresh().getValueAsDouble();
        inputs.scorerVoltage = scorerVoltageSignal.refresh().getValueAsDouble();
        inputs.masterPivotVoltage = masterPivotVoltageSignal.refresh().getValueAsDouble();
        inputs.followerPivotVoltage = followerPivotVoltageSignal.refresh().getValueAsDouble();
        inputs.scorerCurrentDraw = scorerCurrentDrawSignal.refresh().getValueAsDouble();
        inputs.masterPivotCurrentDraw = masterPivotCurrentDrawSignal.refresh().getValueAsDouble();
        inputs.followerPivotCurrentDraw = followerPivotCurrentDrawSignal.refresh().getValueAsDouble();
        inputs.scorerTemperature = scorerTemperatureSignal.refresh().getValueAsDouble();
        inputs.masterPivotTemperature = masterPivotTemperatureSignal.refresh().getValueAsDouble();
        inputs.followerPivotTemperature = followerPivotTemperatureSignal.refresh().getValueAsDouble();
    }

    @Override
    public void setMotorVoltageScorer(double voltage) {
        scorer.setVoltage(voltage);
    }

    @Override
    public void setMotorVoltagePivot(double voltage) {
        masterPivot.setVoltage(voltage);
    }

    @Override
    public void resetPosition() {
        masterPivot.setPosition(0);
        followerPivot.setPosition(0);
    }

    @Override
    public void stop() {
        scorer.setControl(stopRequest);
        masterPivot.setControl(stopRequest);
        followerPivot.setControl(stopRequest);
    }
}
