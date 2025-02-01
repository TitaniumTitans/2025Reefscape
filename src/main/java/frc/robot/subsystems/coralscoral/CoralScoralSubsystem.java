package frc.robot.subsystems.coralscoral;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralScoralIO;
import frc.robot.subsystems.CoralScoralIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import java.sql.Time;

public class CoralScoralSubsystem extends SubsystemBase {
    private final CoralScoralIO io;
    private final CoralScoralIOInputsAutoLogged inputs;
    private final Timer reverseTimer = new Timer();
    public CoralScoralSubsystem(CoralScoralIO io) {
        this.io = io;
        inputs = new CoralScoralIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral Scoral", inputs);
    }

    public void setScorerPower(double voltage) {
        if (voltage < 0.0 && !reverseTimer.isRunning()) {
            reverseTimer.start();
        } else if (voltage < 0.0 && reverseTimer.hasElapsed(1.0)) {
            voltage = MathUtil.clamp(voltage, 0.0, 1.0);
        }

        io.setMotorVoltageScorer(voltage);
    }

    public void setPivotPower(double power) {
        io.setMotorVoltagePivot(power);
    }

    public Command setScorerPowerFactory(double scorerPower) {
        return runEnd(() -> setScorerPower(scorerPower), () -> setScorerPower(0.0));
    }

    public Command setPivotPowerFactory(double pivotPower) {
        return run(() -> setPivotPower(pivotPower));
    }
}
