package frc.robot.subsystems.intake;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSimulation implements IntakeIO {
    private final DCMotorSim intakeSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(1), 0.001, IntakeConstants.INTAKE_GEAR_RATIO
            ),
            DCMotor.getKrakenX60Foc(1)
    );
    private final DCMotorSim pivotSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(1), 0.001, IntakeConstants.PIVOT_GEAR_RATIO
            ),
            DCMotor.getKrakenX60Foc(1)
    );
    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        // update simulation
        intakeSim.update(IntakeConstants.LOOP_PERIOD_SECS);
        pivotSim.update(IntakeConstants.LOOP_PERIOD_SECS);

        // update inputs
        //inputs.intakeVoltage = in
    }
    @Override
    public void setMotorVoltageIntake(double voltage) {
        intakeSim.setInputVoltage(voltage);
    }
    @Override
    public void setMotorVoltagePivot(double voltage) {
        pivotSim.setInputVoltage(voltage);
    }
}
