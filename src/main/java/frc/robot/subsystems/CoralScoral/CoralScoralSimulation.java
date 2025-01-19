package frc.robot.subsystems.CoralScoral;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.CoralScoralIO;
import frc.robot.subsystems.CoralScoralIOInputsAutoLogged;

public class CoralScoralSimulation implements CoralScoralIO {
    private final TalonFX scorer = new TalonFX(CoralScoralConstants.SCORER_ID);
    private final TalonFX masterPivot = new TalonFX(CoralScoralConstants.MASTER_PIVOT_ID);
    private final TalonFX followerPivot = new TalonFX(CoralScoralConstants.FOLLOWER_PIVOT_ID);
    private final TalonFXSimState scorerSimState = scorer.getSimState();
    private final TalonFXSimState masterPivotSimState = masterPivot.getSimState();
    private final TalonFXSimState followerPivotSimState = followerPivot.getSimState();

    private final DCMotorSim scorerMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(1), 0.001, CoralScoralConstants.SCORER_GEAR_RATIO),
                    DCMotor.getKrakenX60Foc(1)
            );
    private final DCMotorSim pivotMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(2), 0.001, CoralScoralConstants.SCORER_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(2)
    );

    @Override
    public void updateInputs(CoralScoralIOInputsAutoLogged inputs) {
        scorerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        masterPivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        followerPivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        scorerMotorSim.update(CoralScoralConstants.LOOP_PERIOD_SECS);
        pivotMotorSim.update(CoralScoralConstants.LOOP_PERIOD_SECS);

        scorerSimState.setRawRotorPosition(
                Units.radiansToRotations(scorerMotorSim.getAngularPositionRad()));
        scorerSimState.setRotorVelocity(
                Units.radiansToRotations(scorerMotorSim.getAngularVelocityRadPerSec()));
        masterPivotSimState.setRawRotorPosition(
                Units.radiansToRotations(pivotMotorSim.getAngularPositionRad()));
        masterPivotSimState.setRotorVelocity(
                Units.radiansToRotations(pivotMotorSim.getAngularVelocityRadPerSec()));
        followerPivotSimState.setRawRotorPosition(
                Units.radiansToRotations(pivotMotorSim.getAngularPositionRad()));
        followerPivotSimState.setRotorVelocity(
                Units.radiansToRotations(pivotMotorSim.getAngularVelocityRadPerSec()));

        inputs.pivotPosition = Rotation2d.fromRotations(masterPivot.getPosition().getValueAsDouble());
        inputs.pivotVelocity = masterPivot.getVelocity().getValueAsDouble();
        inputs.scorerVelocity = scorer.getVelocity().getValueAsDouble();
        inputs.scorerVoltage = scorer.getMotorVoltage().getValueAsDouble();
        inputs.masterPivotVoltage = masterPivot.getMotorVoltage().getValueAsDouble();
        inputs.followerPivotVoltage = followerPivot.getMotorVoltage().getValueAsDouble();
        inputs.scorerCurrentDraw = scorer.getSupplyCurrent().getValueAsDouble();
        inputs.masterPivotCurrentDraw = masterPivot.getSupplyCurrent().getValueAsDouble();
        inputs.followerPivotCurrentDraw = followerPivot.getSupplyCurrent().getValueAsDouble();
        inputs.scorerTemperature = scorer.getDeviceTemp().getValueAsDouble();
        inputs.masterPivotTemperature = masterPivot.getDeviceTemp().getValueAsDouble();
        inputs.followerPivotTemperature = followerPivot.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setMotorVoltageScorer(double voltage) {
        scorer.setVoltage(voltage);
    }

    @Override
    public void setMotorVoltagePivot(double voltage) {
        masterPivot.setVoltage(voltage);
    }
}

