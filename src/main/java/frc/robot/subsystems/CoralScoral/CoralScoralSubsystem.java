package frc.robot.subsystems.CoralScoral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralScoralIO;
import frc.robot.subsystems.CoralScoralIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class CoralScoralSubsystem extends SubsystemBase {
    private final CoralScoralIO io;
    private final CoralScoralIOInputsAutoLogged inputs;
    public CoralScoralSubsystem(CoralScoralIO io) {
        this.io = io;
        inputs = new CoralScoralIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral Scoral", inputs);
    }

    public void setScorerPower(double power) {
        io.setMotorVoltageScorer(power);
    }

    public void setPivotPower(double power) {
        io.setMotorVoltagePivot(power);
    }

    public Command setScorerPowerFactory(double scorerPower) {
        return run(() -> setScorerPower(scorerPower));
    }

    public Command setPivotPowerFactory(double pivotPower) {
        return run(() -> setPivotPower(pivotPower));
    }
}
