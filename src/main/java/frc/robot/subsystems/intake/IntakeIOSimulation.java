package frc.robot.subsystems.intake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeIOSimulation implements IntakeIO {
    private final TalonFX intake = new TalonFX(IntakeConstants.INTAKE_ID);
    private final TalonFX pivot = new TalonFX(IntakeConstants.PIVOT_ID);
    private final TalonFXSimState intakeSimState = intake.getSimState();
    private final TalonFXSimState pivotSimState = pivot.getSimState();

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
        intakeSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        // update simulation
        intakeMotorSim.update(IntakeConstants.LOOP_PERIOD_SECS);
        pivotMotorSim.update(IntakeConstants.LOOP_PERIOD_SECS);

        // update inputs
        intakeMotorSim.setInputVoltage(intakeSimState.getMotorVoltage());
        pivotMotorSim.setInputVoltage(pivotSimState.getMotorVoltage());

        intakeSimState.setRawRotorPosition(
                Units.radiansToRotations(intakeMotorSim.getAngularPositionRad()));
        intakeSimState.setRotorVelocity(
                Units.radiansToRotations(intakeMotorSim.getAngularVelocityRadPerSec()));
        pivotSimState.setRawRotorPosition(
                Units.radiansToRotations(pivotMotorSim.getAngularPositionRad()));
        pivotSimState.setRotorVelocity(
                Units.radiansToRotations(pivotMotorSim.getAngularVelocityRadPerSec()));

        //inputs.intakeVoltage = in
        inputs.pivotPosititon = Rotation2d.fromRotations(pivot.getPosition().getValueAsDouble());
        inputs.intakeVoltage = intake.getMotorVoltage().getValueAsDouble();
        inputs.pivotVoltage = pivot.getMotorVoltage().getValueAsDouble();
        inputs.intakeCurrentDraw = intake.getSupplyCurrent().getValueAsDouble();
        inputs.pivotCurrentDraw = pivot.getSupplyCurrent().getValueAsDouble();
        inputs.intakeTemperature = intake.getDeviceTemp().getValueAsDouble();
        inputs.pivotTemperature = pivot.getDeviceTemp().getValueAsDouble();
        inputs.intakeVelocity = intake.getVelocity().getValueAsDouble();
        inputs.pivotVelocity = pivot.getVelocity().getValueAsDouble();
    }
    @Override
    public void setMotorVoltageIntake(double voltage) {
        intake.setVoltage(voltage);
    }
}
