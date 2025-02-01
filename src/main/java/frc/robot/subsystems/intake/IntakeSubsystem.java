package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MechanismVisualizer;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        MechanismVisualizer.getInstance().setIntakeAngle(inputs.pivotPosititon.getDegrees());
    }

    public void setIntakePower(double power) {
        io.setMotorVoltageIntake(power);
    }

    public void setPivotPower(double power) {
        io.setMotorVoltagePivot(power);
    }

    public Command runIntake(double intakePower) {
        return runEnd(() -> setIntakePower(intakePower), ()-> setIntakePower(-0.5));
    }

    public Command runPivot(double pivotPower) {
        return runEnd(() -> setPivotPower(pivotPower), () -> setPivotPower(0.0));
    }
}
