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

        setPivotPosition(100);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        MechanismVisualizer.getInstance().setIntakeAngle(inputs.pivotPosititon.getDegrees());

        if (inputs.limitSwitch) {
            io.zeroPivot();
        }
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
        return runEnd(
            () -> setPivotPosition(angleDegrees),
            () -> setPivotPosition(inputs.pivotPosititon.getDegrees()));
    }

    public Command pickUpAlgea() {
        return runEnd(() -> {
            setIntakePower(-12.0);
            setPivotPosition(37.5);
        }, () -> {
            setPivotPosition(75);
            setIntakePower(-1.0);
        }).until(() -> inputs.hasAlgae);
    }

    public Command dropAlgae() {
        return runEnd(() -> {
            setIntakePower(3.75);
            setPivotPosition(75);
        }, () -> {
            setPivotPosition(100);
            setIntakePower(0.0);
        });
    }

    public Command zeroPivot() {
        return run(() -> setPivotPower(0.5))
            .until(() -> inputs.limitSwitch)
            .andThen(runOnce(() -> setPivotPower(0.0))
            .andThen(runOnce(io::zeroPivot)))
            .andThen(() -> io.setPivotAngle(100));
    }

    public Command zeroPivot(double power) {
        return run(() -> setPivotPower(power))
            .until(() -> inputs.limitSwitch)
            .andThen(runOnce(() -> setPivotPower(0.0))
                .andThen(runOnce(io::zeroPivot)))
            .andThen(() -> io.setPivotAngle(100));
    }
}
