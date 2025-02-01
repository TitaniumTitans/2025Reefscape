package frc.robot.subsystems.coralscoral;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

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

//    private final LaserCan[] lidars;

    public CoralScoralIOTalon() {
        scorer = new TalonFX(CoralScoralConstants.SCORER_ID);
        masterPivot = new TalonFX(CoralScoralConstants.MASTER_PIVOT_ID);
        followerPivot = new TalonFX(CoralScoralConstants.FOLLOWER_PIVOT_ID);

        TalonFXConfiguration scorerConfig = new TalonFXConfiguration();
        scorerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        scorerConfig.CurrentLimits.SupplyCurrentLimit = 70;
        scorerConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        scorerConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        scorerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        scorerConfig.CurrentLimits.StatorCurrentLimit = 120;
        scorerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        scorer.getConfigurator().apply(scorerConfig);

        TalonFXConfiguration masterPivotConfig = new TalonFXConfiguration();
        masterPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        masterPivotConfig.CurrentLimits.SupplyCurrentLimit = 70;
        masterPivotConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        masterPivotConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        masterPivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        masterPivotConfig.CurrentLimits.StatorCurrentLimit = 100;
        masterPivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterPivotConfig.Feedback.SensorToMechanismRatio = CoralScoralConstants.PIVOT_GEAR_RATIO;

        masterPivot.getConfigurator().apply(masterPivotConfig);
        followerPivot.getConfigurator().apply(masterPivotConfig);

//        lidars = new LaserCan[]{
//            new LaserCan(CoralScoralConstants.LIDAR_IDS[0]),
//            new LaserCan(CoralScoralConstants.LIDAR_IDS[1]),
//            new LaserCan(CoralScoralConstants.LIDAR_IDS[2]),
//            new LaserCan(CoralScoralConstants.LIDAR_IDS[3]),
//        };

        pivotFollowerRequest = new Follower(CoralScoralConstants.MASTER_PIVOT_ID, false);

        var scorerConfigs = new Slot0Configs();
        scorerConfigs.kP = 0.0;
        scorerConfigs.kI = 0.0;
        scorerConfigs.kD = 0.0;
        scorer.getConfigurator().apply(scorerConfigs);

        var pivotConfigs = new Slot0Configs();
        pivotConfigs.kP = 0.0;
        pivotConfigs.kI = 0.0;
        pivotConfigs.kD = 0.0;
        masterPivot.getConfigurator().apply(pivotConfigs);

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

//        for (int i = 0; i < 4; i++) {
//            var measurement = lidars[i].getMeasurement();
//
//            if (measurement.status == 0) {
//                inputs.hasCoral[i] = measurement.distance_mm < 5000;
//            }
//        }
    }

    @Override
    public void setMotorVoltageScorer(double voltage) {
        scorer.setVoltage(voltage);
    }

    @Override
    public void setMotorVoltagePivot(double voltage) {
        masterPivot.setVoltage(voltage);
        followerPivot.setControl(pivotFollowerRequest);
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
