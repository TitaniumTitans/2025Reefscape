package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs;
    private boolean climberLock = false;
    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
        inputs = new ClimberIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }
    public void setClimberPower(double power) {
        io.setMotorVoltage(power);
    }
    public boolean getClimberLock() {
        return climberLock;
    }
    public Command resetClimberLock() {
        return runOnce(() -> climberLock = false).ignoringDisable(true);
    }
    public Command setClimberPowerFactory(double power) {
        return runEnd(() -> setClimberPower(power),
            () -> setClimberPower(0.0));
    }
    public Command setClimberPosition(double degrees) {
        climberLock = true;
        return runEnd(() -> {
            io.setPosititon(degrees);
        },
                () -> setClimberPower(0.0));
    }
    public Command resetClimber() {
        return runOnce(io::resetPosition);
    }
}
