package frc.robot.subsystems.coralscoral;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class CoralScoralSubsystem extends SubsystemBase {
    private final CoralScoralIO io;
    private final CoralScoralIOInputsAutoLogged inputs;
    private final Timer reverseTimer = new Timer();

    @Getter
    @AutoLogOutput(key="Coral Scoral/Has Coral")
    private boolean hasCoral = false;

    public CoralScoralSubsystem(CoralScoralIO io) {
        this.io = io;
        inputs = new CoralScoralIOInputsAutoLogged();

        AutoLogOutputManager.addObject(this);
        setScorerPower(0.2);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral Scoral", inputs);

        hasCoral = false;
        for (double range : inputs.coralRanges) {
            hasCoral |= range < 30;
        }
    }

    public void setScorerPower(double voltage) {
        io.setMotorVoltageScorer(voltage);
    }

    public void setPivotPower(double power) {
        io.setMotorVoltagePivot(power);
    }

    public Command setScorerPowerFactory(double scorerPower) {
        return runEnd(() -> setScorerPower(scorerPower), () -> setScorerPower(0.2));
    }

    public Command setPivotPowerFactory(double pivotPower) {
        return runEnd(() -> {
            double power = pivotPower;
            if (inputs.pivotPosition.getDegrees() > 60.0) {
                power = MathUtil.clamp(power, -12.0, 0.0);
            }

            setPivotPower(power);
            },
            () -> io.setPivotAngle(inputs.pivotPosition.getDegrees()));
    }

    public Command holdPositionFactory() {
        return run(() -> {
            if (inputs.pivotPosition.getDegrees() > 60) {
                io.setMotorVoltagePivot(-0.25);
            } else {
                io.setMotorVoltagePivot(0.0);
            }
        });
    }
}
