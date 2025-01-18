package frc.robot.subsystems.intake;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSimulation implements IntakeIO {
    private final TalonFX intake = new TalonFX(IntakeConstants.INTAKE_ID);
    private final TalonFX pivot = new TalonFX(IntakeConstants.PIVOT_ID);
    private final TalonFXSimState intakeSim = intake.getSimState();
    private final TalonFXSimState pivotSim = pivot.getSimState();
    private final DCMotorSim intakeMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(1), 0.001, IntakeConstants.INTAKE_GEAR_RATIO
            ),
            DCMotor.getKrakenX60Foc(1)
    );
    private final DCMotorSim pivotMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(1), 0.001, IntakeConstants.PIVOT_GEAR_RATIO
            ),
            DCMotor.getKrakenX60Foc(1)
    );

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        // update simulation
        intakeMotorSim.update(IntakeConstants.LOOP_PERIOD_SECS);
        pivotMotorSim.update(IntakeConstants.LOOP_PERIOD_SECS);

        // update inputs
        intakeSim.setSupplyVoltage(intake.getMotorVoltage().getValueAsDouble());
        pivotSim.setSupplyVoltage(pivot.getMotorVoltage().getValueAsDouble());
        intakeMotorSim.setInputVoltage(intakeSim.getMotorVoltage());
        pivotMotorSim.setInputVoltage(pivotSim.getMotorVoltage());

        //inputs.intakeVoltage = in
        inputs.intakeVoltage = intake.getMotorVoltage().refresh().getValueAsDouble();
        inputs.pivotVoltage = pivot.getMotorVoltage().refresh().getValueAsDouble();
        inputs.

    }
}
