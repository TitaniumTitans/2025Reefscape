package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.MechanismVisualizer;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();

//        setPivotPosition(90);
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

    public void setPivotPosition(double angleDegrees) {
        io.setPivotAngle(angleDegrees);
    }

    public Command runIntake(double intakePower) {
        return runEnd(() -> setIntakePower(intakePower), ()-> setIntakePower(-0.0));
    }

    public Command runPivot(double pivotPower) {
        return runEnd(() -> setPivotPower(pivotPower), () -> setPivotPower(0.0));
    }

    public Command setPivotPositionFactory(double angleDegrees) {
        return runEnd(() -> {
            setPivotPosition(angleDegrees);
        }, () -> {
            setPivotPosition(inputs.pivotPosititon.getDegrees());
        });
    }

    public Command pickUpAlgea() {
        return runEnd(() -> {
            setIntakePower(-12.0);
            setPivotPosition(35);
        }, () -> {
            setPivotPosition(75);
            setIntakePower(-0.5);
        });
    }

    public Command dropAlgea() {
        return runEnd(() -> {
            setIntakePower(6.0);
            setPivotPosition(75);
        }, () -> {
            setPivotPosition(90);
            setIntakePower(0.0);
        });
    }

    public Command zeroPivot() {
        return run(() -> setPivotPower(-1.0))
            .until(() -> inputs.limitSwitch)
            .andThen(run(io::zeroPivot));
    }
}
