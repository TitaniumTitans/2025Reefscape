package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    }

    public void setIntakePower(double power) {
        io.setMotorVoltageIntake(power);
    }

    public void setPivotPower(double power) {
        io.setMotorVoltagePivot(power);
    }

    public Command runIntake(double intakePower) {
        return run(() -> setIntakePower(intakePower));
    }

    public Command runPivot(double pivotPower) {
        return run(() -> setPivotPower(pivotPower));
    }
}
